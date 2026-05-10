# Battery MQTT Setup

This setup keeps battery telemetry completely separate from the current ROS2 and Zenoh bridge stack.

- The `INA260` estimator runs on the LattePanda.
- It publishes retained battery JSON over `MQTT`.
- The coverage planner on the production laptop reads that MQTT state with `mosquitto_sub`.
- No ROS topic, launch file, or Zenoh bridge change is required.

## Current Defaults

The updated `SOC_est.py` publishes by default to:

- broker host: `127.0.0.1`
- broker port: `1883`
- state topic: `pilot/battery/state`
- availability topic: `pilot/battery/availability`

The coverage planner code expects the same default topic and port unless you override them in `QSettings`.

## Robot Setup

### 1. Install the broker and Python dependency

On the LattePanda:

```bash
sudo apt update
sudo apt install -y mosquitto mosquitto-clients python3-pip
python3 -m pip install --user paho-mqtt
```

If the robot image does not already have the Adafruit INA260 stack:

```bash
python3 -m pip install --user adafruit-blinka adafruit-circuitpython-ina260
```

### 2. Configure Mosquitto to listen on the robot network

Create `/etc/mosquitto/conf.d/pilot-battery.conf`:

```conf
listener 1883 0.0.0.0
allow_anonymous true
persistence false
```

Then restart the broker:

```bash
sudo systemctl restart mosquitto
sudo systemctl enable mosquitto
```

If the LattePanda firewall is enabled, allow the MQTT port from the Microhard side.

### 3. Place the estimator script in the workspace

Keep the script in the workspace so pathing is consistent across machines:

```bash
~/pilot_ws/src/pilot_control/scripts/battery_soc_monitor_kf.py
```

If your workspace root differs, adjust the `ExecStart` path in the unit file below.

### 4. Test the script manually on the robot

Run it locally first:

```bash
SOC_MQTT_HOST=127.0.0.1 \
SOC_MQTT_PORT=1883 \
SOC_MQTT_TOPIC_STATE=pilot/battery/state \
SOC_MQTT_TOPIC_AVAILABILITY=pilot/battery/availability \
python3 ~/pilot_ws/src/pilot_control/scripts/battery_soc_monitor_kf.py
```

In another terminal on the robot:

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t pilot/battery/state -v
```

You should see JSON payloads with fields like:

- `soc_percent`
- `voltage_v`
- `current_a`
- `power_w`
- `warn`
- `critical`
- `updated_at_ms`
- `stale_after_ms`

### 5. Install the battery service

Create `/etc/systemd/system/battery-soc.service`:

```ini
[Unit]
Description=BDR Battery SOC Monitor
After=multi-user.target
Wants=network-online.target
After=network-online.target

[Service]
Type=simple
User=roofus
WorkingDirectory=/home/roofus/pilot_ws/src/pilot_control/scripts

Environment=BLINKA_MCP2221=1
Environment=SOC_MQTT_ENABLED=1
Environment=SOC_MQTT_HOST=127.0.0.1
Environment=SOC_MQTT_PORT=1883
Environment=SOC_MQTT_TOPIC_STATE=pilot/battery/state
Environment=SOC_MQTT_TOPIC_AVAILABILITY=pilot/battery/availability

ExecStart=/usr/bin/python3 /home/roofus/pilot_ws/src/pilot_control/scripts/battery_soc_monitor_kf.py

Restart=always
RestartSec=3

# Gives the service a clean shutdown window so it can save state.
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
```

Then enable it:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now battery-soc.service
```

Check status:

```bash
systemctl status battery-soc.service
journalctl -u battery-soc.service -f
```

## Laptop Setup

### 1. Install the MQTT client utility

On the production laptop:

```bash
sudo apt update
sudo apt install -y mosquitto-clients
```

The current coverage planner integration uses `mosquitto_sub` via `QProcess`, so `mosquitto-clients` must be installed on the laptop.

### 2. Validate the network path before launching the planner

From the laptop:

```bash
mosquitto_sub -h <robot_ip> -p 1883 -t pilot/battery/state -v
```

You should immediately receive the retained battery message once the robot broker is reachable.

### 3. Launch the coverage planner

The updated planner reads battery MQTT independently of ROS2 and the Zenoh bridge. It will show battery status in the `Point Cloud & Network` panel.

If you keep the defaults, no extra planner configuration is needed:

- topic: `pilot/battery/state`
- port: `1883`

## Changing Topic Or Port Later

If you later want a different topic or port, keep the robot and laptop consistent:

- robot `systemd` environment must match
- planner settings must match

The planner currently loads these optional `QSettings` keys:

- `battery_mqtt_topic`
- `battery_mqtt_port`

## Notes

- This battery path is independent of `robot_complete.launch.py`.
- This battery path is independent of the current Zenoh DDS bridge.
- The only shared resource is the Microhard IP link.
- Keep publish rate low, around `1 Hz`, to make the traffic negligible.
