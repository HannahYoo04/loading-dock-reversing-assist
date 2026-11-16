// 48033 Final Project - Loading Dock Reversing Assist (Bay 1)
//
// An HC-SR04 on the dock wall measures the distance to a reversing truck.
// The ESP32 guides the driver with traffic lights, a beeping buzzer and an LCD,
// and opens the dock door (servo) once the truck has stopped at the dock.
// If the truck leaves while the door is open an alarm is raised
//
// Local:  dock manager's phone/laptop  <->  web app served by the ESP32
// Global: ESP32 -> Wi-Fi -> MQTT broker -> Telegraf -> InfluxDB -> Grafana

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include "webpage.h"
#include "secrets.h"   // WiFi and the MQTT command key

// ---------- pins ----------
#define pin_trig       18   // HC-SR04 trigger
#define pin_echo       34   // HC-SR04 echo
#define pin_thermistor 35   // ADC1
#define pin_buzzer     25   // passive buzzer
#define pin_servo      32   // dock door servo signal
#define pin_led_red    23
#define pin_led_yellow 22
#define pin_led_green  21
#define pin_lcd_sda    13
#define pin_lcd_scl    14

// ---------- WiFi ----------
// WIFI_SSID and WIFI_PASSWORD saved in include/secrets.h
const char *ssid = WIFI_SSID;
const char *pwd  = WIFI_PASSWORD;

// ---------- MQTT broker ----------
const char *mqtt_broker   = "broker.emqx.io";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int   mqtt_port     = 1883;

// ---------- MQTT topics ----------
// group ID / system / bay / data
// Telegraf subscribes to each bay's telemetry, event and status topics and writes them into InfluxDB
const char *topic_telemetry = "jon48033/dock/bay1/telemetry";   // every second: distance, speed, state, door, temp
const char *topic_event     = "jon48033/dock/bay1/event";       // event driven: arrived, docked, creep, released, departed, aborted, settings
const char *topic_status    = "jon48033/dock/bay1/status";      // online when connected, offline if we drop (last will); broker keeps the latest status for new subscribers
const char *topic_command   = "jon48033/dock/bay1/cmd";         // subscribe: release or settings from the office
const char *command_key     = COMMAND_KEY;                      // because it's a public broker ignore commands without the key

const char *online_message  = "{\"online\":1}";
const char *offline_message = "{\"online\":0}";

// ---------- ultrasonic ----------
const unsigned long ECHO_TIMEOUT_US = 15000;  // 15000 us at ~343 m/s is ~5 m round trip, 
const float OUT_OF_RANGE_CM  = 250.0;         // so anything further than 2.5 m is "no object detected"
const unsigned long PING_INTERVAL_MS = 60;    // HC-SR04 datasheet: wait at least 60 ms between pings
const int   MEDIAN_SIZE      = 5;             // number of pings for median filter
const int   SPEED_WINDOW     = 16;            // speed = how fast the distance is changing over the last 16 pings (~1 s)
const int   SPEED_MIN_POINTS = 6;             // only need 6 pings with an echo to work out speed, so a fast truck is caught early
const bool  PLOT_EVERY_PING  = false;         // true = send every ping to Teleplot (testing); false = once a second

// speed of sound depends on air temperature: v = 331.3 + 0.606 * T   (m/s, T in C)
const float SOUND_SPEED_0C   = 331.3;
const float SOUND_SPEED_PER_C = 0.606;

// ---------- thermistor ----------
const float SERIES_RESISTOR_K = 10.0;
const float NOMINAL_R_K       = 10.0;
const float NOMINAL_TEMP_C    = 25.0;
const float B_VALUE           = 3950.0;  // thermistor beta
const float KELVIN_OFFSET     = 273.15;
const int   NUM_SAMPLES       = 10;      // number of samples to average
const float ADC_MAX           = 4095.0;  // 12-bit ADC, (2^12)-1 = 4095
const float SUPPLY_VOLTAGE    = 3.3;
const float VOLTAGE_LIMIT     = 3.29;    // to avoid divide by 0
const float TEMP_MIN_C        = 0.0;     // outside 0-50 C the thermistor is probably loose or shorted
const float TEMP_MAX_C        = 50.0;
const float TEMP_FALLBACK_C   = 20.0;    // if the thermistor reading looks faulty, fall back to 20 C

// ---------- dock zones ----------
float detectDistanceCm = 65.0;    // truck approaching (green)
float slowDistanceCm   = 40.0;    // slow down (yellow)
float stopDistanceCm   = 10.0;    // stop (red)

const float ZONE_HYSTERESIS_CM = 10.0;   // to avoid jitter, a truck must be 10 cm past detect distance to count as gone
const float STILL_SPEED_CMS    = 3.0;    // "stopped" velocity
const unsigned long DOCK_HOLD_MS = 2000; // door-open threshold [time]
const float CREEP_CM           = 5.0;    // creep-alarm threshold [distance]
const unsigned long CREEP_HOLD_MS = 300; // creep-alarm threshold [time]
const float STOP_LINE_SLACK_CM = 1.0;    // stop-line jitter allowance [distance]
const float HARD_DOCK_SPEED_CMS = 20.0;  // hard-dock threshold [speed]
const float BUMPER_CM          = 4.0;    // the bumper is 4 cm past the door
const unsigned long ARRIVE_HOLD_MS = 300; // arrive threshold [time]
const float BRAKE_WARN_CMS2    = 30.0;   // STOP NOW threshold [deceleration]
const float TTC_MAX_S          = 10.0;   // time-to-contact cap [time]

// ---------- dock door servo ----------
// 50 Hz -> 20 ms period. 0.5 ms high = 0 deg, 2.5 ms high = 180 deg.
// 10-bit = 1024 steps. 0.5/20 * 1024 = ~25, 1.5/20 * 1024 = ~77 (90 deg)
const int SERVO_CHANNEL    = 2;     // LEDC timer 1
const int SERVO_FREQ_HZ    = 50;
const int SERVO_RESOLUTION = 10;
const int DOOR_CLOSED_DUTY = 25;    // 0 degrees
const int DOOR_OPEN_DUTY   = 77;    // 90 degrees

// ---------- buzzer ----------
const int BUZZER_CHANNEL    = 0;      // LEDC timer 0
const int BUZZER_RESOLUTION = 10;
const int GUIDE_TONE_HZ     = 2000;   // reversing beeps
const int ALARM_TONE_HZ     = 3000;   // creep alarm
const int BEEP_ON_MS        = 80;
const int BEEP_INTERVAL_FAR_MS   = 800;    // interval between beeps at the detect distance
const int BEEP_INTERVAL_NEAR_MS  = 80;     // interval between beeps at the stop distance
const int ALARM_ON_MS       = 100;
const int ALARM_OFF_MS      = 100;

// ---------- timing ----------
const unsigned long TEMP_INTERVAL_MS      = 2000;   // air temperature changes slowly
const unsigned long LCD_INTERVAL_MS       = 250;    // I2C is slow, we do not want to redraw every loop
const unsigned long PUBLISH_INTERVAL_MS   = 1000;
const unsigned long RECONNECT_INTERVAL_MS = 5000;
const unsigned long DNS_RETRY_MS          = 60000;  // look up the broker's IP again at most once a minute
const unsigned long WIFI_WAIT_MS          = 10000;  // at power on, wait up to 10 s for WiFi, then run the dock without it
const unsigned long FLASH_INTERVAL_MS     = 500;    // flashing green light when released
const unsigned long LCD_SWAP_MS           = 3000;   // empty bay: alternate temperature and IP address every 3 s
const int NETWORK_TIMEOUT_S = 1;                    // timeout broker connection attempt after 1 s
const int MQTT_BUFFER_SIZE  = 512;                  // PubSubClient default is 256 bytes for the whole packet; an event can exceed that (topic + up to 250 B JSON)
const time_t YEAR_2024 = 1704067200;                // any time before this means the NTP clock is not set yet

// ---------- dock states ----------
const int STATE_EMPTY       = 0;
const int STATE_APPROACHING = 1;
const int STATE_DOCKED      = 2;
const int STATE_RELEASED    = 3;
const int STATE_CREEP       = 4;

int dockState = STATE_EMPTY;

// ---------- measurements ----------
float medianBuffer[MEDIAN_SIZE];   // buffer for raw distances
int   medianIndex = 0;
float rawDistanceCm = OUT_OF_RANGE_CM;
float distanceCm  = OUT_OF_RANGE_CM;   // filtered distance
float speedDistances[SPEED_WINDOW];          // buffer for filtered distances...
unsigned long speedTimes[SPEED_WINDOW];      // ...and when each one was measured (ms)
int   speedIndex  = 0;
int   speedCount  = 0;
float speedCms    = 0.0;           // approach speed from a straight-line fit over the window (+ = towards the dock)
float speed2PointCms = 0.0;        // speed from just the first and last ping, for Teleplot comparison
float timeToContactS = TTC_MAX_S;  // seconds until the truck reaches the bumper at its current speed
float brakeNeededCms2 = 0.0;       // braking needed to stop before the bumper, v^2 / (2 * distance left)
float temperatureC = 25.0;

// ---------- per-truck records ----------
float maxApproachSpeedCms = 0.0;   // fastest speed seen inside the slow zone
float minDistanceCm       = OUT_OF_RANGE_CM;
float dockedDistanceCm    = 0.0;
bool  hardDock            = false;
bool  stopNowWarning      = false;
unsigned long nearSince   = 0;     // when something first came inside the detect distance (0 = nothing near)
unsigned long stillSince  = 0;     // when the truck first stopped inside the stop zone (0 = not still)
unsigned long creepSince  = 0;     // when the truck first moved past the creep distance (0 = it has not)
unsigned long dockedTime  = 0;     // when the door opened
unsigned long releasedTime = 0;    // when the door closed
int dockCount     = 0;
int hardDockCount = 0;
bool doorOpen     = false;

// ---------- timers ----------
unsigned long lastPingTime      = 0;
unsigned long lastTempTime      = 0;
unsigned long lastLcdTime       = 0;
unsigned long lastPublishTime   = 0;
unsigned long lastReconnectAttempt  = 0;
unsigned long lastDnsTry        = 0;
unsigned long lastBeepChange    = 0;
unsigned long lastFlashTime     = 0;
bool buzzerOn  = false;
bool flashOn   = false;

// ---------- data rate (W10 s4-10) ----------
unsigned long bytesSent       = 0;   // MQTT bytes since the last data rate report
unsigned long lastRateTime    = 0;

String clientID;
IPAddress brokerIP;              // looked up once, so a reconnect does not wait for DNS
bool brokerIPKnown = false;
bool wifiWasConnected = false;

// Library objects
WiFiClient espClient;
PubSubClient client(espClient);
WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2);      // address 0x27, 16 columns, 2 rows
Preferences preferences;                 // saves stop/slow distances to flash (NVS) so they survive a power cut

// Forward declarations
void reconnect();
void publishEvent(const char *eventName);
bool releaseTruck();

// State variable to string conversions
const char *stateName(int state)
{
  if (state == STATE_APPROACHING)
  {
    return "APPROACHING";
  }
  else if (state == STATE_DOCKED)
  {
    return "DOCKED";
  }
  else if (state == STATE_RELEASED)
  {
    return "RELEASED";
  }
  else if (state == STATE_CREEP)
  {
    return "CREEP ALARM";
  }
  return "EMPTY";
}

// ============================================================
//  Sensors
// ============================================================

// Convert thermistor voltage into degrees
float readTemperature()
{
  // Averaging to reduce noise
  long total = 0;
  for (int i = 0; i < NUM_SAMPLES; i++)
  {
    total += analogRead(pin_thermistor);
  }
  float adcValue = (float)total / NUM_SAMPLES;

  // Convert ADC value to voltage
  float voltage = adcValue / ADC_MAX * SUPPLY_VOLTAGE;

  // Divide-by-zero guard
  if (voltage > VOLTAGE_LIMIT)
  {
    voltage = VOLTAGE_LIMIT;
  }

  // Calculate the thermistor resistance using the voltage divider formula
  float thermistorK = SERIES_RESISTOR_K * voltage / (SUPPLY_VOLTAGE - voltage);

  // B-value formula: 1/T = 1/T1 + ln(Rt/R) / B       [in Kelvin]
  float nominalTempK    = NOMINAL_TEMP_C + KELVIN_OFFSET;
  float resistanceRatio = thermistorK / NOMINAL_R_K;
  float lnRatio         = log(resistanceRatio);
  float inverseTempK    = 1.0 / nominalTempK + lnRatio / B_VALUE;
  float tempK           = 1.0 / inverseTempK;
  float tempC           = tempK - KELVIN_OFFSET;

  // A loose thermistor reads about -66 C and a shorted one about -273 C.
  // Fall back to a normal room temperature if fault detected
  if (isnan(tempC) || tempC < TEMP_MIN_C || tempC > TEMP_MAX_C)
  {
    Serial.printf("Thermistor reads %.1f C, check the wiring. Using %.0f C\n", tempC, TEMP_FALLBACK_C);
    return TEMP_FALLBACK_C;
  }
  return tempC;
}

// One HC-SR04 ping
// Returns the distance in cm, or OUT_OF_RANGE_CM if no echo came back.
float pingDistance()
{
  // Step 1: a 10 us HIGH pulse on TRIG starts a measurement
  digitalWrite(pin_trig, LOW);
  delayMicroseconds(2);
  digitalWrite(pin_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trig, LOW);

  // Step 2 and 3: pulseIn() waits for ECHO to go HIGH and times how long it stays HIGH
  unsigned long echoMicros = pulseIn(pin_echo, HIGH, ECHO_TIMEOUT_US);

  // 0 means the timeout ran out: nothing within range
  if (echoMicros == 0)
  {
    return OUT_OF_RANGE_CM;
  }

  // Step 4: time -> distance.
  // Speed of sound corrected for the air temperature from the thermistor.
  float soundSpeedMs = SOUND_SPEED_0C + SOUND_SPEED_PER_C * temperatureC;

  // distance = speed * time / 2   (divide by 2 because the sound goes there AND back)
  // us * m/s = micrometres, so divide by 10000 to get cm
  float distance = echoMicros * soundSpeedMs / 10000.0 / 2.0;
  return distance;
}

// Median of the last 5 pings to filter outliers
float medianDistance()
{
  // copy the buffer so sorting does not mess up the order of readings
  float sorted[MEDIAN_SIZE];
  for (int i = 0; i < MEDIAN_SIZE; i++)
  {
    sorted[i] = medianBuffer[i];
  }

  // bubble sort, smallest to largest
  for (int i = 0; i < MEDIAN_SIZE - 1; i++)
  {
    for (int j = 0; j < MEDIAN_SIZE - 1 - i; j++)
    {
      if (sorted[j] > sorted[j + 1])
      {
        float temp = sorted[j];
        sorted[j] = sorted[j + 1];
        sorted[j + 1] = temp;
      }
    }
  }

  // the middle value of 5 sorted values is sorted[2]
  return sorted[MEDIAN_SIZE / 2];
}

// Take a new ping, store it, update the filtered distance
void updateDistance()
{
  rawDistanceCm = pingDistance();

  medianBuffer[medianIndex] = rawDistanceCm;
  medianIndex = (medianIndex + 1) % MEDIAN_SIZE;   // wrap around

  distanceCm = medianDistance();
}

// Calculate approach speed from the last 16 filtered distances (~1 s).
// Fits a straight line (least squares) through the (time, distance) points to calculate the speed.
// Uses every point, so one noisy reading moves it far less than a first-to-last estimate.
void updateSpeed()
{
  // store the newest distance and when it was measured
  speedDistances[speedIndex] = distanceCm;
  speedTimes[speedIndex] = millis();
  speedIndex = (speedIndex + 1) % SPEED_WINDOW;   // wrap around, speedIndex now points at the oldest reading
  if (speedCount < SPEED_WINDOW)
  {
    speedCount++;
  }

  // Only fit the readings where something was in range. 
  // Before a truck arrives the window is full of 250 cm "nothing there" readings, 
  // and a jump from 250 to 40 cm is not a real speed.
  // The window is a circular buffer: read it oldest first, starting at 0 before it fills, then at speedIndex.
  int oldestIndex = (speedIndex + SPEED_WINDOW - speedCount) % SPEED_WINDOW;   // oldest reading = next write position minus how many readings we have, wrapped around
  unsigned long oldestTime = speedTimes[oldestIndex];   // times in seconds from here, so the numbers stay small
  int points = 0;
  int firstIndex = -1;
  int lastIndex = -1;
  float meanTime = 0.0;
  float meanDistance = 0.0;
  for (int n = 0; n < speedCount; n++)
  {
    int i = (oldestIndex + n) % SPEED_WINDOW;
    if (speedDistances[i] >= OUT_OF_RANGE_CM)
    {
      continue;
    }
    if (firstIndex < 0)
    {
      firstIndex = i;
    }
    lastIndex = i;
    meanTime += (speedTimes[i] - oldestTime) / 1000.0;
    meanDistance += speedDistances[i];
    points++;
  }

  if (points < SPEED_MIN_POINTS)
  {
    speedCms = 0.0;
    speed2PointCms = 0.0;
    return;
  }
  meanTime = meanTime / points;
  meanDistance = meanDistance / points;

  // slope = sum((t - mean t) * (d - mean d)) / sum((t - mean t)^2)
  float top = 0.0;
  float bottom = 0.0;
  for (int n = 0; n < speedCount; n++)
  {
    int i = (oldestIndex + n) % SPEED_WINDOW;
    if (speedDistances[i] >= OUT_OF_RANGE_CM)
    {
      continue;
    }
    float t = (speedTimes[i] - oldestTime) / 1000.0 - meanTime;
    float d = speedDistances[i] - meanDistance;
    top += t * d;
    bottom += t * t;
  }
  if (bottom > 0)
  {
    speedCms = -top / bottom;   // distance shrinks as the truck approaches, so flip the sign to make approaching positive
  }

  // first-to-last estimate over the same readings, for the Teleplot comparison
  float seconds = (speedTimes[lastIndex] - speedTimes[firstIndex]) / 1000.0;
  if (seconds > 0)
  {
    speed2PointCms = (speedDistances[firstIndex] - speedDistances[lastIndex]) / seconds;
  }
}

// Time to contact = distance left to the bumper / approach speed (shown on the web app and sent to the cloud).
// Braking needed = v^2 / (2 * distance left), from v^2 = u^2 + 2as with the truck ending at 0.
// The braking needed is only high when the truck is too fast for the distance left.
void updateTimeToContact()
{
  if (speedCms > STILL_SPEED_CMS && distanceCm > BUMPER_CM)
  {
    float distanceLeft = distanceCm - BUMPER_CM;
    timeToContactS = distanceLeft / speedCms;
    if (timeToContactS > TTC_MAX_S)
    {
      timeToContactS = TTC_MAX_S;
    }
    brakeNeededCms2 = speedCms * speedCms / (2.0 * distanceLeft);
  }
  else
  {
    timeToContactS = TTC_MAX_S;   // stopped, reversing away, or already at the bumper
    brakeNeededCms2 = 0.0;
  }
}

bool stopNow()
{
  return dockState == STATE_APPROACHING && brakeNeededCms2 > BRAKE_WARN_CMS2;
}

// ============================================================
//  Outputs
// ============================================================

void setDoor(bool open)
{
  if (open)
  {
    ledcWrite(SERVO_CHANNEL, DOOR_OPEN_DUTY);
  }
  else
  {
    ledcWrite(SERVO_CHANNEL, DOOR_CLOSED_DUTY);
  }
  doorOpen = open;
}

void setLights(bool red, bool yellow, bool green)
{
  digitalWrite(pin_led_red, red);
  digitalWrite(pin_led_yellow, yellow);
  digitalWrite(pin_led_green, green);
}

// Mute the buzzer and log the state
void buzzerOff()
{
  ledcWriteTone(BUZZER_CHANNEL, 0);
  buzzerOn = false;
}

// Non-blocking beep pattern: on for onTime, off for offTime, repeat
void beep(int toneHz, int onTime, int offTime)
{
  unsigned long now = millis();
  if (buzzerOn && now - lastBeepChange >= onTime)
  {
    buzzerOff();
    lastBeepChange = now;
  }
  else if (!buzzerOn && now - lastBeepChange >= offTime)
  {
    ledcWriteTone(BUZZER_CHANNEL, toneHz);
    buzzerOn = true;
    lastBeepChange = now;
  }
}

// Traffic lights + buzzer, decided by the state and the distance
void updateGuidance()
{
  if (stopNow())
  {
    // Approaching too fast to stop before the bumper: red + alarm, no matter what zone it is in
    setLights(true, false, false);
    beep(ALARM_TONE_HZ, ALARM_ON_MS, ALARM_OFF_MS);
  }
  else if (dockState == STATE_APPROACHING)
  {
    // Lights: green far out, yellow in the slow zone, red at the stop line
    if (distanceCm <= stopDistanceCm)
    {
      setLights(true, false, false);
    }
    else if (distanceCm <= slowDistanceCm)
    {
      setLights(false, true, false);
    }
    else
    {
      setLights(false, false, true);
    }

    // Parking-sensor beeps: the closer the truck, the shorter the interval between beeps.
    float safeDistance = constrain(distanceCm, stopDistanceCm, detectDistanceCm); // clamp so map() stays between near and far intervals
    int intervalMs = map(safeDistance, stopDistanceCm, detectDistanceCm, BEEP_INTERVAL_NEAR_MS, BEEP_INTERVAL_FAR_MS);
    beep(GUIDE_TONE_HZ, BEEP_ON_MS, intervalMs);
  }
  else if (dockState == STATE_DOCKED)
  {
    // Solid red: do not move while loading
    setLights(true, false, false);
    if (buzzerOn)
    {
      buzzerOff();
    }
  }
  else if (dockState == STATE_CREEP)
  {
    // Flashing red + fast alarm
    if (millis() - lastFlashTime >= ALARM_ON_MS)
    {
      lastFlashTime = millis();
      flashOn = !flashOn;
    }
    setLights(flashOn, false, false);
    beep(ALARM_TONE_HZ, ALARM_ON_MS, ALARM_OFF_MS);
  }
  else if (dockState == STATE_RELEASED)
  {
    // Flashing green: free to drive away
    if (millis() - lastFlashTime >= FLASH_INTERVAL_MS)
    {
      lastFlashTime = millis();
      flashOn = !flashOn;
    }
    setLights(false, false, flashOn);
    if (buzzerOn)
    {
      buzzerOff();
    }
  }
  else   // EMPTY
  {
    // Solid green: bay is free
    setLights(false, false, true);
    if (buzzerOn)
    {
      buzzerOff();
    }
  }
}

// Seconds the current truck has been at the dock (0 if no truck docked)
unsigned long dwellSeconds()
{
  if (dockState == STATE_DOCKED || dockState == STATE_CREEP)
  {
    return (millis() - dockedTime) / 1000;
  }
  else if (dockState == STATE_RELEASED)
  {
    return (releasedTime - dockedTime) / 1000;
  }
  return 0;
}

// Updates both LCD lines for the driver.
// Each line is padded to 16 characters so a shorter message fully covers the old one.
// This avoids lcd.clear(), which blanks the screen and would flicker at every 250 ms update.
void updateLcd()
{
  char line1[17];   // 16 characters + the '\0' at the end
  char line2[17];

  if (dockState == STATE_APPROACHING)
  {
    if (stopNow())
    {
      snprintf(line1, sizeof(line1), "!! STOP NOW !!  ");
    }
    else if (distanceCm <= stopDistanceCm)
    {
      snprintf(line1, sizeof(line1), "STOP     %5.1fcm", distanceCm);
    }
    else if (distanceCm <= slowDistanceCm)
    {
      snprintf(line1, sizeof(line1), "SLOW     %5.1fcm", distanceCm);
    }
    else
    {
      snprintf(line1, sizeof(line1), "REVERSE  %5.1fcm", distanceCm);
    }
    snprintf(line2, sizeof(line2), "Speed %5.1fcm/s ", speedCms);
  }
  else if (dockState == STATE_DOCKED)
  {
    snprintf(line1, sizeof(line1), "DOCKED  DOOR OPN");
    snprintf(line2, sizeof(line2), "Loading %5lus  ", dwellSeconds());
  }
  else if (dockState == STATE_CREEP)
  {
    snprintf(line1, sizeof(line1), "!! STOP TRUCK !!");
    snprintf(line2, sizeof(line2), "Moved %5.1fcm   ", distanceCm - dockedDistanceCm);
  }
  else if (dockState == STATE_RELEASED)
  {
    snprintf(line1, sizeof(line1), "DOOR CLOSED     ");
    snprintf(line2, sizeof(line2), "OK TO DEPART    ");
  }
  else
  {
    snprintf(line1, sizeof(line1), "BAY 1 FREE      ");
    // alternate between the air temperature and the address of the dock manager web app
    if ((millis() / LCD_SWAP_MS) % 2 == 0)
    {
      snprintf(line2, sizeof(line2), "Air %4.1fC       ", temperatureC);
    }
    else if (WiFi.status() == WL_CONNECTED)
    {
      snprintf(line2, sizeof(line2), "%-16s", WiFi.localIP().toString().c_str());
    }
    else
    {
      snprintf(line2, sizeof(line2), "No WiFi         ");
    }
  }

  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

// ============================================================
//  Dock state machine
// ============================================================

void setState(int newState)
{
  // if state has not changed, return
  if (newState == dockState)
  {
    return;
  }
  Serial.printf("State: %s -> %s\n", stateName(dockState), stateName(newState));
  dockState = newState;
  buzzerOff();   // every state starts its own beep pattern from silence
}

void startApproach()
{
  // reset the records for this truck
  maxApproachSpeedCms = 0.0;
  minDistanceCm = OUT_OF_RANGE_CM;
  hardDock = false;
  stopNowWarning = false;
  stillSince = 0;
  setState(STATE_APPROACHING);
}

void dockTruck()
{
  dockedTime = millis();
  dockedDistanceCm = distanceCm;
  creepSince = 0;
  dockCount++;

  // hard dock = approached the slow zone too fast, or hit the bumper
  if (maxApproachSpeedCms > HARD_DOCK_SPEED_CMS || minDistanceCm < BUMPER_CM)
  {
    hardDock = true;
    hardDockCount++;
  }

  setDoor(true);
  setState(STATE_DOCKED);
  publishEvent("docked");
}

// Called from the web app Release button
// Returns false if there is no docked truck to release
bool releaseTruck()
{
  // no truck docked
  if (dockState != STATE_DOCKED && dockState != STATE_CREEP)
  {
    return false;
  }

  // record the time the door closed and release the truck
  releasedTime = millis();
  setDoor(false);
  setState(STATE_RELEASED);
  publishEvent("released");
  return true;
}

void updateDockState()
{
  if (dockState == STATE_EMPTY)
  {
    // something must stay inside the detect distance for 0.3 s,
    // so a few stray readings or a hand passing the sensor are not counted as a truck
    if (distanceCm < detectDistanceCm)
    {
      if (nearSince == 0)
      {
        nearSince = millis();
      }
      else if (millis() - nearSince >= ARRIVE_HOLD_MS)
      {
        nearSince = 0;
        startApproach();
        publishEvent("arrived");
      }
    }
    else
    {
      nearSince = 0;
    }
  }
  else if (dockState == STATE_APPROACHING)
  {
    // record this truck's closest distance, top speed in the slow zone, and any STOP NOW warning
    if (distanceCm < minDistanceCm)
    {
      minDistanceCm = distanceCm;
    }
    if (distanceCm <= slowDistanceCm && speedCms > maxApproachSpeedCms)
    {
      maxApproachSpeedCms = speedCms;
    }
    if (stopNow())
    {
      stopNowWarning = true;
    }

    if (distanceCm > detectDistanceCm + ZONE_HYSTERESIS_CM)
    {
      // truck drove off without docking
      setState(STATE_EMPTY);
      publishEvent("aborted");
    }
    else if ((distanceCm <= stopDistanceCm ||
              (stillSince != 0 && distanceCm <= stopDistanceCm + STOP_LINE_SLACK_CM)) &&
             abs(speedCms) < STILL_SPEED_CMS)
    {
      // stopped inside the stop line: start the 2 s door timer, or keep it running.
      // Once it's running, readings up to 1 cm outside the line don't reset it.
      if (stillSince == 0)
      {
        stillSince = millis();
      }
      else if (millis() - stillSince >= DOCK_HOLD_MS)
      {
        dockTruck();
      }
    }
    else
    {
      stillSince = 0;   // moved again, restart the timer
    }
  }
  else if (dockState == STATE_DOCKED)
  {
    // truck pulled away from the dock with the door still open.
    // It must stay out for 0.3 s to trigger the alarm. If 3 of the last 5 pings miss their echo (250 cm),
    // the median jumps to 250 for a moment and would set off a false alarm.
    if (distanceCm > dockedDistanceCm + CREEP_CM)
    {
      if (creepSince == 0)
      {
        creepSince = millis();
      }
      else if (millis() - creepSince >= CREEP_HOLD_MS)
      {
        creepSince = 0;
        setState(STATE_CREEP);
        publishEvent("creep");
      }
    }
    else
    {
      creepSince = 0;
    }
  }
  else if (dockState == STATE_CREEP)
  {
    // truck reversed back onto the dock: clear the alarm once it's within 2.5 cm of where it stopped.
    // Clearing at half the 5 cm trigger stops a truck sitting near 5 cm flipping between DOCKED and CREEP
    // and publishing "creep" over and over.
    if (distanceCm <= dockedDistanceCm + CREEP_CM / 2)
    {
      setState(STATE_DOCKED);
    }
  }
  else if (dockState == STATE_RELEASED)
  {
    if (distanceCm > detectDistanceCm + ZONE_HYSTERESIS_CM)
    {
      setState(STATE_EMPTY);
      publishEvent("departed");
    }
  }
}

// ============================================================
//  MQTT (global path)
// ============================================================

// Current time as milliseconds since 1 Jan 1970 (UTC), from the NTP-synced clock.
// Every message carries this, so InfluxDB stores when it was MEASURED, not when Telegraf received it.
long long epochMillis()
{
  struct timeval now;
  gettimeofday(&now, NULL);
  long long milliseconds = (long long)now.tv_sec * 1000 + now.tv_usec / 1000;
  return milliseconds;
}

// Has NTP set the clock yet? Before it does, the clock starts at 1970.
bool clockIsSet()
{
  return time(nullptr) >= YEAR_2024;
}

// Publish and count the bytes, so we can check our data rate
void publishAndCount(const char *topic, const char *payload)
{
  // A 1970 timestamp would put the point 56 years in the past, where Grafana never looks.
  // The ESP32 keeps retrying NTP in the background, so just wait for it.
  if (!clockIsSet())
  {
    Serial.println("Clock not set by NTP yet, message not sent");
    return;
  }

  // only count what the broker actually received
  if (client.publish(topic, payload))
  {
    bytesSent += strlen(topic) + strlen(payload);
  }
}

// Telemetry once a second, for example:
// {"distance_cm":35.2,"speed_cms":8.1,"ttc_s":10,"temp_c":23.4,"state":"APPROACHING","state_code":1,"door_open":0,"rssi":-52,"ts":1791345600123}
void publishTelemetry()
{
  JsonDocument doc;
  doc["distance_cm"] = round(distanceCm * 10) / 10.0;   // round to 1 decimal place
  doc["speed_cms"]   = round(speedCms * 10) / 10.0;
  doc["ttc_s"]       = round(timeToContactS * 10) / 10.0;
  doc["temp_c"]      = round(temperatureC * 10) / 10.0;
  doc["state"]       = stateName(dockState);
  doc["state_code"]  = dockState;
  doc["door_open"]   = doorOpen ? 1 : 0;
  doc["rssi"]        = WiFi.RSSI();
  doc["ts"]          = epochMillis();

  char buffer[200];
  serializeJson(doc, buffer);
  publishAndCount(topic_telemetry, buffer);
}

// One message per event, so Grafana can count and list them, for example:
// {"event":"docked","state_code":2,"ts":1791345600123,"max_speed_cms":12.5,"min_distance_cm":8.1,"hard_dock":0,"stop_now_warning":0,"dock_count":3}
// Every event carries state_code: Telegraf makes "event" a tag, and InfluxDB needs at least one number per point.
void publishEvent(const char *eventName)
{
  JsonDocument doc;
  doc["event"]      = eventName;
  doc["state_code"] = dockState;
  doc["ts"]         = epochMillis();

  if (strcmp(eventName, "docked") == 0)
  {
    doc["max_speed_cms"]   = round(maxApproachSpeedCms * 10) / 10.0;
    doc["min_distance_cm"] = round(minDistanceCm * 10) / 10.0;
    doc["hard_dock"]       = hardDock ? 1 : 0;
    doc["stop_now_warning"] = stopNowWarning ? 1 : 0;
    doc["dock_count"]      = dockCount;
  }
  else if (strcmp(eventName, "aborted") == 0)
  {
    doc["stop_now_warning"] = stopNowWarning ? 1 : 0;   // a truck can be warned and then drive off
  }
  else if (strcmp(eventName, "settings") == 0)
  {
    doc["stop_cm"] = stopDistanceCm;
    doc["slow_cm"] = slowDistanceCm;
  }
  else if (strcmp(eventName, "released") == 0)
  {
    doc["dwell_s"] = dwellSeconds();
  }
  else if (strcmp(eventName, "creep") == 0)
  {
    doc["creep_cm"] = round((distanceCm - dockedDistanceCm) * 10) / 10.0;
  }

  char buffer[250];
  serializeJson(doc, buffer);
  publishAndCount(topic_event, buffer);
  Serial.print("Event published: ");
  Serial.println(buffer);
}

// Attempt reconnection to the broker
void reconnect()
{
  Serial.print("Attempting MQTT connection...");

  // Look up the broker's IP once and store it. 
  // A DNS lookup can block for up to 15 s, which would freeze the lights, 
  // buzzer and pings if it happened on every reconnect.
  if (!brokerIPKnown)
  {
    if (lastDnsTry != 0 && millis() - lastDnsTry < DNS_RETRY_MS)
    {
      Serial.println("skipped, waiting to look up the broker again");
      return;
    }
    lastDnsTry = millis();
    if (!WiFi.hostByName(mqtt_broker, brokerIP))
    {
      Serial.println("DNS lookup for the broker failed");
      return;
    }
    brokerIPKnown = true;
    client.setServer(brokerIP, mqtt_port);
  }

  // connect(id, user, password, willTopic, willQos, willRetain, willMessage):
  // the last four register our "last will" offline message with the broker.
  if (client.connect(clientID.c_str(), mqtt_username, mqtt_password,
                     topic_status, 0, true, offline_message))
  {
    Serial.println("connected");
    client.publish(topic_status, online_message, true);
    // Clear any retained command first, so an old "release" is not replayed every time we reconnect. 
    // Then re-subscribe because the broker forgets us.
    client.publish(topic_command, "", true);
    client.subscribe(topic_command);
  }
  else
  {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    brokerIPKnown = false;   // the broker may have moved, look it up again next time
  }
}

// Check new dock zones and apply them (from the web app or an MQTT command).
// Saved to flash, so a reboot keeps them.
// Returns NULL if they were applied, or the reason they were rejected.
const char *applySettings(float newStop, float newSlow)
{
  // sanity check: stop line must be in front of the slow zone, slow zone inside the detect range.
  // Checks the values are good rather than looking for bad ones, so NaN is rejected too.
  bool ok = newStop >= 2 && newSlow >= newStop + 5 && newSlow <= detectDistanceCm - 5;
  if (!ok)
  {
    return "Need 2 <= stop, stop + 5 <= slow, slow <= 60";
  }

  stopDistanceCm = newStop;
  slowDistanceCm = newSlow;
  preferences.putFloat("stop", stopDistanceCm);
  preferences.putFloat("slow", slowDistanceCm);
  Serial.printf("Settings changed: stop %.1f cm, slow %.1f cm\n", stopDistanceCm, slowDistanceCm);
  publishEvent("settings");
  return NULL;
}

// MQTT command from the operations office, for example:
// {"cmd":"release","key":"..."}  or  {"cmd":"settings","stop":10,"slow":40,"key":"..."}
void onMqttMessage(char *topic, byte *payload, unsigned int length)
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  if (error)
  {
    Serial.print("Bad command JSON: ");
    Serial.println(error.c_str());
    return;
  }

  const char *command = doc["cmd"] | "";   // "" if there is no cmd key
  Serial.printf("MQTT command: %s\n", command);

  // the broker is public, so anyone could publish here: ignore commands without our key
  const char *key = doc["key"] | "";
  if (strcmp(key, command_key) != 0)
  {
    Serial.println("Command ignored: wrong or missing key");
    return;
  }

  if (strcmp(command, "release") == 0)
  {
    if (!releaseTruck())
    {
      Serial.println("Release ignored: no truck docked");
    }
  }
  else if (strcmp(command, "settings") == 0)
  {
    // -1 if a value is missing, which applySettings rejects
    const char *problem = applySettings(doc["stop"] | -1.0f, doc["slow"] | -1.0f);
    if (problem != NULL)
    {
      Serial.printf("Settings rejected: %s\n", problem);
    }
  }
  else
  {
    Serial.println("Unknown command");
  }
}

// ============================================================
//  Web server (local path)
// ============================================================

// GET /  -> the dock manager web app
void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

// GET /status -> live values as JSON, read by the web app every 500 ms
void handleStatus()
{
  JsonDocument doc;
  doc["distance_cm"]     = distanceCm;
  doc["speed_cms"]       = speedCms;
  doc["ttc_s"]           = timeToContactS;
  doc["brake_cms2"]      = brakeNeededCms2;
  doc["stop_now"]        = stopNow();
  doc["temp_c"]          = temperatureC;
  doc["state"]           = stateName(dockState);
  doc["state_code"]      = dockState;
  doc["door_open"]       = doorOpen;
  doc["dwell_s"]         = dwellSeconds();
  doc["dock_count"]      = dockCount;
  doc["hard_dock_count"] = hardDockCount;
  doc["stop_cm"]         = stopDistanceCm;
  doc["slow_cm"]         = slowDistanceCm;

  String text;
  serializeJson(doc, text);
  server.send(200, "application/json", text);
}

// POST /release -> close the door, truck may leave
void handleRelease()
{
  if (releaseTruck())
  {
    server.send(200, "text/plain", "Door closed, truck released");
  }
  else
  {
    server.send(409, "text/plain", "No truck docked");   // 409 = request does not fit the current state
  }
}

// POST /settings with stop=..&slow=.. -> change the dock zones
void handleSettings()
{
  if (!server.hasArg("stop") || !server.hasArg("slow"))
  {
    server.send(400, "text/plain", "Missing stop or slow");   // 400 = bad request
    return;
  }

  // toFloat() gives 0 for text that is not a number, which applySettings rejects
  const char *problem = applySettings(server.arg("stop").toFloat(), server.arg("slow").toFloat());
  if (problem != NULL)
  {
    server.send(400, "text/plain", problem);
    return;
  }
  server.send(200, "text/plain", "Settings saved");
}

// ============================================================

void setup()
{
  Serial.begin(115200);

  // ultrasonic
  pinMode(pin_trig, OUTPUT);
  pinMode(pin_echo, INPUT);
  digitalWrite(pin_trig, LOW);

  // traffic lights
  pinMode(pin_led_red, OUTPUT);
  pinMode(pin_led_yellow, OUTPUT);
  pinMode(pin_led_green, OUTPUT);
  setLights(false, false, false);

  // buzzer hardware pwm setup
  ledcSetup(BUZZER_CHANNEL, GUIDE_TONE_HZ, BUZZER_RESOLUTION);
  ledcAttachPin(pin_buzzer, BUZZER_CHANNEL);
  buzzerOff();

  // servo hardware pwm setup, door starts closed
  ledcSetup(SERVO_CHANNEL, SERVO_FREQ_HZ, SERVO_RESOLUTION);
  ledcAttachPin(pin_servo, SERVO_CHANNEL);
  setDoor(false);

  // LCD
  Wire.begin(pin_lcd_sda, pin_lcd_scl);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Dock bay 1");
  lcd.setCursor(0, 1);
  lcd.print("Connecting WiFi");

  // fill the median buffer with "nothing there"
  for (int i = 0; i < MEDIAN_SIZE; i++)
  {
    medianBuffer[i] = OUT_OF_RANGE_CM;
  }
  temperatureC = readTemperature();

  // dock zones saved from the last run
  preferences.begin("dock", false);
  stopDistanceCm = preferences.getFloat("stop", stopDistanceCm);
  slowDistanceCm = preferences.getFloat("slow", slowDistanceCm);
  Serial.printf("Dock zones: stop %.1f cm, slow %.1f cm\n", stopDistanceCm, slowDistanceCm);

  // Wait up to 10 s for WiFi. The guidance (lights, buzzer, LCD, door) must work without it,
  // so after that the dock runs anyway and the ESP32 keeps reconnecting in the background.
  WiFi.begin(ssid, pwd);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < WIFI_WAIT_MS)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\n");

  // Get the real time from the internet (NTP).
  // 0, 0 = no timezone offset: we send UTC and let Grafana show local time.
  // If WiFi is not up yet, the ESP32 keeps trying in the background once it is.
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  lcd.setCursor(0, 1);
  if (WiFi.status() == WL_CONNECTED)
  {
    wifiWasConnected = true;
    Serial.print("Connected to the Wi-Fi network with local IP: ");
    Serial.println(WiFi.localIP());

    Serial.print("Waiting for NTP time");
    int tries = 0;
    while (!clockIsSet() && tries < 20)   // timeout after 10 s, publishAndCount() skips sending until NTP sets the clock
    {
      delay(500);
      Serial.print(".");
      tries++;
    }
    Serial.println();

    // show the IP so the dock manager knows which address to open
    lcd.print(WiFi.localIP().toString() + "    ");
  }
  else
  {
    Serial.println("No WiFi yet, running the dock without it");
    lcd.print("No WiFi         ");
  }

  // web server: which function runs for which address
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/release", HTTP_POST, handleRelease);
  server.on("/settings", HTTP_POST, handleSettings);
  server.begin();

  // A unique client ID from the MAC address
  clientID = "jon48033-dock-";
  clientID += WiFi.macAddress();

  // Timeout connection attempt after 1s (setServer is called in reconnect(), with the broker's IP)
  espClient.setTimeout(NETWORK_TIMEOUT_S);
  client.setSocketTimeout(NETWORK_TIMEOUT_S);
  client.setBufferSize(MQTT_BUFFER_SIZE);
  client.setCallback(onMqttMessage);   // runs inside client.loop() when a command arrives
  if (WiFi.status() == WL_CONNECTED)
  {
    reconnect();
  }

  Serial.println("Dock bay 1 ready.");
  delay(2000);   // leave the IP on the LCD for 2 s
}

void loop()
{
  // WiFi reconnected
  bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (wifiConnected && !wifiWasConnected)
  {
    Serial.print("WiFi connected, local IP: ");
    Serial.println(WiFi.localIP());
  }
  wifiWasConnected = wifiConnected;

  // If the broker connection dropped, try again 5 s after the last attempt
  // A try can block for up to ~2 s, so wait for idle state
  // the lights, beeps and alarm must keep running while a truck is moving.
  bool truckMoving = dockState == STATE_APPROACHING || dockState == STATE_CREEP;
  if (!client.connected() && wifiConnected && !truckMoving)
  {
    if (millis() - lastReconnectAttempt >= RECONNECT_INTERVAL_MS)
    {
      buzzerOff();   // turn off the buzzer before a blocking connection attempt
      reconnect();
      lastReconnectAttempt = millis();   // set after the attempt, so there are always 5 s free between attempts even if one is slow
    }
  }

  client.loop();          // MQTT keep-alive
  server.handleClient();  // answer any web app requests

  // ping the ultrasonic sensor every 60 ms and run the state machine on the new distance
  if (millis() - lastPingTime >= PING_INTERVAL_MS)
  {
    lastPingTime = millis();
    updateDistance();
    updateSpeed();
    updateTimeToContact();
    updateDockState();

    if (PLOT_EVERY_PING)
    {
      Serial.printf(">raw:%.1f\n", rawDistanceCm);
      Serial.printf(">distance:%.1f\n", distanceCm);
      Serial.printf(">speed:%.1f\n", speedCms);
      Serial.printf(">speed_2pt:%.1f\n", speed2PointCms);
      Serial.printf(">ttc:%.2f\n", timeToContactS);
      Serial.printf(">brake:%.1f\n", brakeNeededCms2);
    }
  }

  if (millis() - lastTempTime >= TEMP_INTERVAL_MS)
  {
    lastTempTime = millis();
    temperatureC = readTemperature();
  }

  updateGuidance();   // lights and buzzer

  if (millis() - lastLcdTime >= LCD_INTERVAL_MS)
  {
    lastLcdTime = millis();
    updateLcd();
  }

  if (millis() - lastPublishTime >= PUBLISH_INTERVAL_MS)
  {
    lastPublishTime = millis();
    publishTelemetry();
    // Teleplot traces for bench testing
    if (!PLOT_EVERY_PING)
    {
      Serial.printf(">distance:%.1f\n", distanceCm);
      Serial.printf(">speed:%.1f\n", speedCms);
    }
    Serial.printf(">state:%d\n", dockState);

    // average MQTT data rate since the last report, in bytes per second
    float seconds = (millis() - lastRateTime) / 1000.0;
    float bytesPerSecond = bytesSent / seconds;
    Serial.printf(">mqtt_bytes_per_s:%.1f\n", bytesPerSecond);
    bytesSent = 0;
    lastRateTime = millis();
  }
}
