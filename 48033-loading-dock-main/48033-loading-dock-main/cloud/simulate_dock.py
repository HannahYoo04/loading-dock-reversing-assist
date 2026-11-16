# 48033 Final Project - fake dock node for testing the cloud side without the hardware
# Simulated dock node for testing the cloud services without the ESP32.
# Publishes the same MQTT topics and JSON as dock_node/src/main.cpp, so Telegraf,
# InfluxDB and Grafana can be built and tested before the ESP32 is wired up.
#
# Install the required package: 
# pip install paho-mqtt
# Backfill 2 hours of past trucks (sample data), takes about 2 min: 
# python simulate_dock.py --backfill 120  
# Send one truck after another, in real time
# python simulate_dock.py --live         
#
# It publishes as bay "sim" unless --bay is given, so fake trucks never mix with the real bay1.
# Use --bay to publish under a different bay name.
# Each backfill message includes a simulated timestamp (ts). Telegraf uses it
# so InfluxDB stores the readings in the past where they "happened".

import argparse
import json
import random
import time

import paho.mqtt.client as mqtt

BROKER = "broker.emqx.io"
PORT = 1883

# Keep the same zones and thresholds in line with the firmware
DETECT_CM = 65.0
SLOW_CM = 40.0
STOP_CM = 10.0
BUMPER_CM = 4.0
HARD_DOCK_SPEED_CMS = 20.0
BRAKE_WARN_CMS2 = 30.0
OUT_OF_RANGE_CM = 250.0

BACKFILL_TELEMETRY_EVERY = 10

# Numeric state codes sent with telemetry and messages
STATE_CODES = {"EMPTY": 0, "APPROACHING": 1, "DOCKED": 2, "RELEASED": 3, "CREEP ALARM": 4}


class FakeDock:
    def __init__(self, client, bay, live):
        self.client = client
        self.live = live
        self.topic = "jon48033/dock/" + bay + "/"
        self.dock_count = 0
        self.telemetry_count = 0
        self.temp_c = random.uniform(18, 26)

    def publish(self, kind, payload):
        self.client.publish(self.topic + kind, json.dumps(payload))
        if self.live:
            print(kind, payload)
        else:
            # broker.emqx.io drops messages above roughly 8 per second from one client (measured)
            time.sleep(0.15)

    def telemetry(self, t, distance, speed, state, door_open):
        if speed > 3 and distance > BUMPER_CM and state == "APPROACHING":
            ttc = min((distance - BUMPER_CM) / speed, 10.0)
        else:
            ttc = 10.0
        self.temp_c += random.uniform(-0.05, 0.05)
        # Backfill: send only every tenth telemetry sample to reduce broker traffic
        self.telemetry_count += 1
        if not self.live and self.telemetry_count % BACKFILL_TELEMETRY_EVERY != 0:
            return
        self.publish("telemetry", {
            # The firmware uses exactly 250 cm to represent an out-of-range reading
            "distance_cm": OUT_OF_RANGE_CM if distance >= OUT_OF_RANGE_CM else round(distance + random.gauss(0, 0.3), 1),
            "speed_cms": round(speed + random.gauss(0, 0.5), 1),
            "ttc_s": round(ttc, 1),
            "temp_c": round(self.temp_c, 1),
            "state": state,
            "state_code": STATE_CODES[state],
            "door_open": door_open,
            "rssi": random.randint(-65, -45),
            "ts": int(t * 1000),
        })

    def event(self, t, name, state, **extra):
        payload = {"event": name, "state_code": STATE_CODES[state], "ts": int(t * 1000)}
        payload.update(extra)
        self.publish("event", payload)

    def wait(self, t, seconds):
        # Live mode waits (sleep) in real time. backfill: only advances the simulated clock
        if self.live:
            time.sleep(seconds)
            return time.time()
        return t + seconds

    def one_truck(self, t):
        # Run one truck through the dock, starting at time t. return its finish time
        # Bay empty for a while
        for _ in range(random.randint(20, 120)):
            self.telemetry(t, OUT_OF_RANGE_CM, 0, "EMPTY", 0)
            t = self.wait(t, 1)

        # Approach: start at the detect line, slow down towards the stop line
        hard = random.random() < 0.2 # Simulate a hard approach for about 20% of the trucks
        start_speed = random.uniform(25, 35) if hard else random.uniform(10, 18)
        distance = DETECT_CM - 1
        self.event(t, "arrived", "APPROACHING")
        max_speed = 0.0
        min_distance = distance
        stop_now_warning = 0
        stop_at = STOP_CM - random.uniform(0, 2) # Vary the stopping point
        step = 0
        while distance > stop_at:
            # In a hard approach, the truck keeps more speed as it gets closer
            fraction = (distance - STOP_CM + 8) / (DETECT_CM - STOP_CM + 8)
            speed = max(start_speed * fraction ** (0.15 if hard else 1.0), 2.0)
            if distance <= SLOW_CM:
                max_speed = max(max_speed, speed)
            # Flag a warning if the truck needs more braking force than the threshold allows, v^2 / 2s
            if speed > 3 and distance > BUMPER_CM and speed * speed / (2 * (distance - BUMPER_CM)) > BRAKE_WARN_CMS2:
                stop_now_warning = 1
            # Update the truck's position in 0.1 s steps (the firmware checks every 60 ms), telemetry once a second
            if step % 10 == 0:
                self.telemetry(t, distance, speed, "APPROACHING", 0)
                t = self.wait(t, 1)
            distance -= speed * 0.1
            min_distance = max(min(min_distance, distance), 1.5)
            step += 1
        if hard and random.random() < 0.5:
            min_distance = random.uniform(2, BUMPER_CM)   # Hit the bumper
        distance = max(min_distance, 2.0)

        # Hold the truck at the dock for two seconds before recording it as docked
        for _ in range(2):
            self.telemetry(t, distance, 0, "APPROACHING", 0)
            t = self.wait(t, 1)
        self.dock_count += 1
        hard_dock = 1 if (max_speed > HARD_DOCK_SPEED_CMS or min_distance < BUMPER_CM) else 0
        docked_at = t
        self.event(t, "docked", "DOCKED", max_speed_cms=round(max_speed, 1),
                   min_distance_cm=round(min_distance, 1), hard_dock=hard_dock,
                   stop_now_warning=stop_now_warning, dock_count=self.dock_count)

        # Opening the door while loading, with the odd creep away from the dock
        dwell = random.randint(60, 300)
        # About 15% of trucks creep during loading and choose a random start time
        creep_at = random.randint(10, dwell - 10) if random.random() < 0.15 else -1
        for second in range(dwell):
            if creep_at >= 0 and 0 <= second - creep_at < 8:
                creep = (second - creep_at + 1) * 1.5
                state = "CREEP ALARM" if creep > 5 else "DOCKED"
                if creep > 5 and second - creep_at == 3:
                    self.event(t, "creep", "CREEP ALARM", creep_cm=round(creep, 1))
                self.telemetry(t, distance + creep, -1.5, state, 1)
            else:
                self.telemetry(t, distance, 0, "DOCKED", 1)
            t = self.wait(t, 1)

        # The dock manager releases the truck and door closes
        self.event(t, "released", "RELEASED", dwell_s=int(t - docked_at))
        for _ in range(random.randint(3, 10)):
            self.telemetry(t, distance, 0, "RELEASED", 0)
            t = self.wait(t, 1)

        # Truck drives away from the dock
        while distance < DETECT_CM + 10:
            self.telemetry(t, distance, -15, "RELEASED", 0)
            distance += 15
            t = self.wait(t, 1)
        self.event(t, "departed", "EMPTY")
        return t


def main():
    parser = argparse.ArgumentParser(description="Fake loading dock node")
    parser.add_argument("--bay", default="sim", help="bay name in the topic (default sim, never use bay1)")
    parser.add_argument("--backfill", type=int, metavar="MINUTES", help="publish this many minutes of past trucks")
    parser.add_argument("--live", action="store_true", help="publish trucks in real time until Ctrl+C")
    args = parser.parse_args()
    if not args.backfill and not args.live:
        parser.error("choose --backfill MINUTES and/or --live")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set("emqx", "public")
    status_topic = "jon48033/dock/" + args.bay + "/status"
    client.will_set(status_topic, json.dumps({"online": 0}), retain=True)   # Same last will as the ESP32
    client.connect(BROKER, PORT)
    client.loop_start()
    client.publish(status_topic, json.dumps({"online": 1}), retain=True)

    if args.backfill:
        dock = FakeDock(client, args.bay, live=False)
        t = time.time() - args.backfill * 60
        trucks = 0
        while t < time.time() - 700:   # One truck takes up to ~10 min, stop before it runs into the future
            t = dock.one_truck(t)
            trucks += 1
        print("Backfilled", trucks, "trucks")

    if args.live:
        dock = FakeDock(client, args.bay, live=True)
        try:
            while True:
                dock.one_truck(time.time())
        except KeyboardInterrupt:
            pass

    # A normal disconnect does not trigger the last will, so publish the offline status here
    client.publish(status_topic, json.dumps({"online": 0}), retain=True)
    time.sleep(1)
    client.loop_stop()
    client.disconnect()


if __name__ == "__main__":
    main()
