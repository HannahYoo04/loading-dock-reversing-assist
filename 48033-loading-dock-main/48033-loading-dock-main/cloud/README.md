# Cloud side: MQTT -> Telegraf -> InfluxDB -> Grafana

Han-na's part. Everything here was tested end to end on 2026-10-08 against a local InfluxDB 2.7 and Grafana 11.2, using `simulate_dock.py` as the ESP32. `dashboard_simulated.png` is what it looked like.

## Files

- `telegraf.conf`: subscribes to every bay on `broker.emqx.io` and writes into the `dock` bucket.
- `grafana_dashboard.json`: the dashboard. Import it, don't build it by hand.
- `simulate_dock.py`: a fake dock node. Publishes the same topics and JSON as the ESP32, so the cloud side can be built before the hardware works.
- `dashboard_simulated.png`: screenshot of the dashboard with simulated data.

## 1. InfluxDB Cloud

- Log in to the same InfluxDB Cloud account used in Lab 10. If creating a bucket is refused, the free plan's bucket limit may be reached: delete an old lab bucket first.
- Create a bucket called `dock`.
- Create an API token with write access to `dock` (for Telegraf), and a read token for Grafana (or one all-access token for both, as in W10 s20).
- Note the organisation name, shown at the top left or in the URL.

## 2. Telegraf

- Download Telegraf for Windows from influxdata.com and unzip it. Tested with 1.32.3.
- In PowerShell, from this `cloud` folder:

```
$env:INFLUX_TOKEN = "<token>"
$env:INFLUX_ORG   = "<organisation>"
<path to>\telegraf.exe --config telegraf.conf
```

- It should print `Connected [tcp://broker.emqx.io:1883]` twice, one for each input.
- Leave it running for the whole demo. Telegraf is the only thing moving data into InfluxDB: if this window is closed, nothing is stored.
- Check: InfluxDB Data Explorer, bucket `dock`, measurement `telemetry`.

What the config does, for the video:

- Two `mqtt_consumer` inputs. One reads telemetry and events, which carry `ts`. The other reads online/offline status, which has no `ts`.
- `json_time_key = "ts"` makes each point's time the moment the ESP32 measured it, not the moment Telegraf received it.
- `tag_keys = ["state", "event"]` keeps the two text values as tags. Telegraf's JSON parser drops text fields, and a message with no number left in it is dropped completely. This was tested: an event with only `event` and `ts` never reached InfluxDB. That's why every event also carries `state_code`.
- `topic_parsing` turns `jon48033/dock/bay1/telemetry` into measurement `telemetry` with tag `bay=bay1`. Adding bay 2 needs no Telegraf change: the `+` in the topic already matches it (W10 s27).

## 3. Test data without the ESP32

```
pip install paho-mqtt
python simulate_dock.py --backfill 120    # 2 hours of past trucks, takes about 2 min
python simulate_dock.py --live            # one truck after another, in real time (Ctrl+C to stop)
```

- Telegraf must be running first.
- It publishes as bay `sim` by default, so fake trucks never land in the real `bay1`. Pick `sim` in the Bay drop-down to see them.
- It sends `online` as a retained message with the same last will as the ESP32, and `offline` when it stops, so Bay status behaves like the real node.
- broker.emqx.io drops messages from one client above roughly 8 per second (measured: 80% loss at 50/s, none at 6/s). Backfill therefore sends telemetry every 10 s instead of every 1 s. The real ESP32 sends about 1 per second, well under the limit.

## 4. Grafana Cloud

- Add the InfluxDB data source exactly as in W10 s18-20: query language Flux, URL `https://us-east-1-1.aws.cloud2.influxdata.com`, organisation, token.
- Dashboards > New > Import > upload `grafana_dashboard.json` > pick that data source.
- The drop-downs at the top: `Bucket` (default `dock`), `Bay` (filled from the data), `per` (bar width for trucks docked).

## What each panel shows

| Panel | From | Why the dock cares |
|---|---|---|
| Bay status | `status.online`, last value in the past 30 days | ONLINE or OFFLINE. The ESP32 only sends this when it connects, so the panel looks back 30 days rather than at the dashboard's time range. OFFLINE comes from the broker's last will when the ESP32 drops off |
| Trucks docked | count of `docked` events | throughput |
| Hard docks | sum of `hard_dock` | trucks that came in too fast or hit the bumper: damage risk |
| STOP NOW warnings | sum of `stop_now_warning` on `docked` and `aborted` | approaches where the truck needed more than 30 cm/s² of braking to stop before the bumper |
| Creep alarms | count of `creep` events | trucks that moved with the door open: safety incidents |
| Average dwell | mean `dwell_s` of `released` | how long loading takes |
| Dock state | `state_code` and `door_open` | timeline of the bay |
| Distance and approach speed | `distance_cm`, `speed_cms` | each reversing approach; the 250 cm "nothing there" readings are filtered out |
| Trucks docked per period | `docked` count per 1m / 10m / 1h | busy times |
| Approach speed per truck | `max_speed_cms` of `docked`, red line at 20 cm/s | which trucks broke the hard-dock speed |
| Dwell time per truck | `dwell_s` of `released` | slow loads stand out |
| Truck log | `docked` events pivoted into a table | one row per truck |
| Air temperature and WiFi signal | `temp_c`, `rssi` | temperature corrects the speed of sound; RSSI shows a weak link |

## Known quirks

- The dock state timeline leaves a gap wherever there is no data for over 15 s, so an offline bay doesn't look live.
- Backfill sends telemetry every 10 s, so an approach shows only a few points and short creeps may not show on the timeline. For screenshots in the report, use `--live` or real data.
- The dashboard refreshes every 5 s, but Telegraf only writes every 5 s, so data can be up to ~10 s behind.
- Telegraf uses QoS 0 with no persistent session. Anything published while Telegraf is not running is lost.
