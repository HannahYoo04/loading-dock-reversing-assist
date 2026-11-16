// Dock manager web app, served by the ESP32 at http://<board IP>/
//
// The web app is stored as one long text string in flash (PROGMEM) using R"rawliteral( ... )
//
// The web app does three things:
//   1. every 500 ms it asks the ESP32 for /status and shows the live values
//      (the next request only starts after the last one finished, and gives up after 1.5 s)
//   2. the Settings form sends new stop and slow distances to /settings
//   3. the Release button tells the ESP32 to close the door and let the truck go

#pragma once

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Dock Bay 1</title>
<style>
  * { box-sizing: border-box; }   /* width: 100% includes the padding, so buttons do not stick out */
  body { font-family: system-ui, sans-serif; margin: 0; padding: 16px; max-width: 480px; background: #f2f2f2; color: #222; }
  h2 { margin: 0 0 12px; }
  .card { background: white; border-radius: 10px; padding: 14px; margin-bottom: 12px; box-shadow: 0 1px 3px rgba(0,0,0,0.15); }
  .state { font-size: 2em; font-weight: bold; text-align: center; padding: 16px; border-radius: 10px; color: white; background: gray; }
  .warn { animation: flash 0.3s infinite alternate; }
  @keyframes flash { from { opacity: 1; } to { opacity: 0.4; } }
  .bar { position: relative; height: 26px; background: #ddd; border-radius: 6px; overflow: hidden; margin-top: 8px; }
  .zone { position: absolute; top: 0; bottom: 0; }
  .marker { position: absolute; top: 0; bottom: 0; width: 4px; background: black; }
  .big { font-size: 1.6em; font-weight: bold; }
  table { width: 100%; border-collapse: collapse; }
  td { padding: 6px 0; border-bottom: 1px solid #eee; }
  td:last-child { text-align: right; font-weight: bold; }
  input { width: 70px; font-size: 1em; padding: 4px; }
  button { font-size: 1.1em; padding: 12px; border: none; border-radius: 8px; width: 100%; color: white; background: #1565c0; }
  #release { background: #2e7d32; }
  #release:disabled { background: #999; }
  label { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
</style>
</head>
<body>
<h2>Loading Dock - Bay 1</h2>
<div id="state" class="state card">connecting...</div>

<div class="card">
  <div class="big" id="distance">-</div>
  <!-- distance bar: dock wall on the left, 100 cm on the right -->
  <div class="bar">
    <div class="zone" id="zoneStop" style="left:0; background:#e53935;"></div>
    <div class="zone" id="zoneSlow" style="background:#fbc02d;"></div>
    <div class="zone" id="zoneGo" style="right:0; background:#43a047;"></div>
    <div class="marker" id="marker"></div>
  </div>
</div>

<div class="card">
  <table>
    <tr><td>Approach speed</td><td id="speed">-</td></tr>
    <tr><td>Time to bumper</td><td id="ttc">-</td></tr>
    <tr><td>Door</td><td id="door">-</td></tr>
    <tr><td>Time at dock</td><td id="dwell">-</td></tr>
    <tr><td>Trucks docked</td><td id="docks">-</td></tr>
    <tr><td>Hard docks</td><td id="hard">-</td></tr>
    <tr><td>Air temperature</td><td id="temp">-</td></tr>
  </table>
</div>

<div class="card">
  <button id="release" onclick="releaseTruck()">Release truck (close door)</button>
</div>

<div class="card">
  <h3 style="margin-top:0">Settings</h3>
  <form onsubmit="saveSettings(); return false;">
    <label>Stop distance (cm) <input id="stop" type="number" min="2" max="55" step="0.5"></label>
    <label>Slow zone starts at (cm) <input id="slow" type="number" min="7" max="60" step="0.5"></label>
    <button type="submit">Save</button>
  </form>
</div>
<p id="msg"></p>

<script>
// background colour for each state code
const stateColours = ["#43a047", "#fb8c00", "#e53935", "#1565c0", "#8b0000"];
const BAR_MAX_CM = 65;    // the bar shows 0 to 65 cm (the detect distance)
const POLL_MS = 500;
const TIMEOUT_MS = 1500;  // no answer in 1.5 s = treat the ESP32 as unreachable
let settingsLoaded = false;

// fetch() that gives up after TIMEOUT_MS, so a hung ESP32 shows "no connection" instead of old values
function fetchWithTimeout(url, options) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), TIMEOUT_MS);
  return fetch(url, Object.assign({ signal: controller.signal }, options))
    .finally(() => clearTimeout(timer));
}

// cm -> percent across the bar
function percent(cm) {
  return Math.min(cm, BAR_MAX_CM) / BAR_MAX_CM * 100;
}

function updateStatus() {
  fetchWithTimeout("/status")
    .then(response => response.json())
    .then(data => {
      const box = document.getElementById("state");
      box.textContent = data.stop_now ? "STOP NOW" : data.state;
      box.style.background = data.stop_now ? "#b71c1c" : stateColours[data.state_code];
      box.classList.toggle("warn", data.stop_now || data.state_code == 4);

      // distance and the zone bar
      const far = data.distance_cm >= 250;
      document.getElementById("distance").textContent = far ? "Nothing in range" : data.distance_cm.toFixed(1) + " cm";
      document.getElementById("zoneStop").style.width = percent(data.stop_cm) + "%";
      document.getElementById("zoneSlow").style.left = percent(data.stop_cm) + "%";
      document.getElementById("zoneSlow").style.width = (percent(data.slow_cm) - percent(data.stop_cm)) + "%";
      document.getElementById("zoneGo").style.width = (100 - percent(data.slow_cm)) + "%";
      document.getElementById("marker").style.left = "calc(" + percent(data.distance_cm) + "% - 2px)";
      document.getElementById("marker").style.display = far ? "none" : "block";

      document.getElementById("speed").textContent = data.speed_cms.toFixed(1) + " cm/s";
      document.getElementById("ttc").textContent = data.ttc_s >= 10 ? "-" : data.ttc_s.toFixed(1) + " s";
      document.getElementById("temp").textContent = data.temp_c.toFixed(1) + " C";
      document.getElementById("door").textContent = data.door_open ? "OPEN" : "CLOSED";
      document.getElementById("dwell").textContent = data.dwell_s + " s";
      document.getElementById("docks").textContent = data.dock_count;
      document.getElementById("hard").textContent = data.hard_dock_count;

      // Release only makes sense while a truck is docked (DOCKED or CREEP)
      document.getElementById("release").disabled = !(data.state_code == 2 || data.state_code == 4);

      // fill the settings boxes once, so we do not overwrite what the user is typing
      if (!settingsLoaded) {
        document.getElementById("stop").value = data.stop_cm;
        document.getElementById("slow").value = data.slow_cm;
        settingsLoaded = true;
      }
    })
    .catch(() => {
      const box = document.getElementById("state");
      box.textContent = "no connection";
      box.style.background = "gray";
      box.classList.remove("warn");
      document.getElementById("release").disabled = true;
    })
    .finally(() => setTimeout(updateStatus, POLL_MS));   // next poll only after this one is done
}

function releaseTruck() {
  fetchWithTimeout("/release", { method: "POST" })
    .then(response => response.text())
    .then(text => { document.getElementById("msg").textContent = text; })
    .catch(() => { document.getElementById("msg").textContent = "Not sent: no connection to the dock"; });
}

function saveSettings() {
  const body = "stop=" + document.getElementById("stop").value +
               "&slow=" + document.getElementById("slow").value;
  fetchWithTimeout("/settings", {
    method: "POST",
    headers: { "Content-Type": "application/x-www-form-urlencoded" },
    body: body
  })
    .then(response => response.text())
    .then(text => { document.getElementById("msg").textContent = text; settingsLoaded = false; })
    .catch(() => { document.getElementById("msg").textContent = "Not saved: no connection to the dock"; });
}

updateStatus();
</script>
</body>
</html>
)rawliteral";
