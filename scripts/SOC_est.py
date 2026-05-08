#!/usr/bin/env python3

import json
import math
import os
import signal
import socket
import time
from pathlib import Path

import board
import adafruit_ina260

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None


# -----------------------------
# User configuration
# -----------------------------

BATTERY_CAPACITY_AH = 20.0

# For "24V Li-ion", this is usually either:
# 6S: full = 25.2V, nominal = 22.2V
# 7S: full = 29.4V, nominal = 25.9V
CELL_COUNT = 6

# If positive INA260 current means battery is discharging, leave +1.
# If your current is negative during discharge, set to -1.
DISCHARGE_CURRENT_SIGN = 1

# Approximate total pack internal resistance in ohms.
# Used to estimate open-circuit voltage under load:
# V_ocv_est = V_measured + I_discharge * R_pack
# For a 20Ah Li-ion pack, 0.04 to 0.12 ohm is a reasonable tuning range.
PACK_INTERNAL_RESISTANCE_OHM = 0.08

# Voltage correction should only be trusted when current is low.
REST_CURRENT_THRESHOLD_A = 1.0

# If voltage is changing rapidly, do not trust voltage-based SOC correction.
STABLE_VOLTAGE_DELTA_THRESHOLD_V = 0.03

# How strongly to blend voltage SOC into coulomb SOC when resting.
# Small value = stable, slow correction.
VOLTAGE_CORRECTION_GAIN = 0.003

# Sampling and persistence
SAMPLE_PERIOD_S = 1.0
STATE_SAVE_PERIOD_S = 15.0
STATE_FILE = Path("/var/tmp/battery_soc_state.json")

# Telemetry / MQTT
MQTT_ENABLED = os.environ.get("SOC_MQTT_ENABLED", "1") != "0"
MQTT_HOST = os.environ.get("SOC_MQTT_HOST", "127.0.0.1")
MQTT_PORT = int(os.environ.get("SOC_MQTT_PORT", "1883"))
MQTT_KEEPALIVE_S = 30
MQTT_QOS = 1
MQTT_CLIENT_ID = os.environ.get(
    "SOC_MQTT_CLIENT_ID",
    f"pilot-battery-{socket.gethostname()}",
)
MQTT_STATE_TOPIC = os.environ.get("SOC_MQTT_TOPIC_STATE", "pilot/battery/state")
MQTT_AVAILABILITY_TOPIC = os.environ.get(
    "SOC_MQTT_TOPIC_AVAILABILITY",
    "pilot/battery/availability",
)
MQTT_USERNAME = os.environ.get("SOC_MQTT_USERNAME")
MQTT_PASSWORD = os.environ.get("SOC_MQTT_PASSWORD")
PAYLOAD_SCHEMA = "pilot-battery-v1"
STALE_AFTER_MS = int(max(3000.0, SAMPLE_PERIOD_S * 2500.0))

# UI warning thresholds. Accuracy tuning can happen later.
WARN_SOC_PERCENT = 25.0
CRITICAL_SOC_PERCENT = 12.0

# Optional low/high clamps
MIN_SOC_PERCENT = 0.0
MAX_SOC_PERCENT = 100.0

STOP_REQUESTED = False


# -----------------------------
# Li-ion open-circuit voltage curve
# -----------------------------
# Approximate Li-ion OCV curve per cell.
# Format: (voltage_per_cell, soc_percent)
# This should be treated as an estimate, not a lab-grade fuel gauge.
LI_ION_OCV_TABLE = [
    (4.20, 100),
    (4.15, 95),
    (4.10, 90),
    (4.05, 85),
    (4.00, 80),
    (3.96, 70),
    (3.92, 60),
    (3.87, 50),
    (3.82, 40),
    (3.77, 30),
    (3.72, 20),
    (3.65, 10),
    (3.50, 5),
    (3.30, 0),
]


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def interp_soc_from_cell_voltage(v_cell):
    """
    Convert estimated open-circuit voltage per cell to SOC percent
    using piecewise-linear interpolation.
    """
    table = LI_ION_OCV_TABLE

    if v_cell >= table[0][0]:
        return 100.0
    if v_cell <= table[-1][0]:
        return 0.0

    for i in range(len(table) - 1):
        v_hi, soc_hi = table[i]
        v_lo, soc_lo = table[i + 1]

        if v_lo <= v_cell <= v_hi:
            t = (v_cell - v_lo) / (v_hi - v_lo)
            return soc_lo + t * (soc_hi - soc_lo)

    return 0.0


def estimate_voltage_soc(pack_voltage_v, discharge_current_a):
    """
    Estimate SOC from measured voltage.

    If the battery is discharging, measured voltage is lower than OCV
    because of internal resistance. Compensate roughly with I * R.
    """
    discharge_current_a = max(0.0, discharge_current_a)
    estimated_ocv_pack_v = pack_voltage_v + discharge_current_a * PACK_INTERNAL_RESISTANCE_OHM
    estimated_ocv_cell_v = estimated_ocv_pack_v / CELL_COUNT
    soc = interp_soc_from_cell_voltage(estimated_ocv_cell_v)

    return clamp(soc, MIN_SOC_PERCENT, MAX_SOC_PERCENT), estimated_ocv_pack_v, estimated_ocv_cell_v


def load_state():
    if not STATE_FILE.exists():
        return None

    try:
        with STATE_FILE.open("r") as f:
            return json.load(f)
    except Exception:
        return None


def save_state(soc_percent, remaining_ah, voltage_v, current_a):
    data = {
        "timestamp": time.time(),
        "soc_percent": soc_percent,
        "remaining_ah": remaining_ah,
        "voltage_v": voltage_v,
        "current_a": current_a,
        "capacity_ah": BATTERY_CAPACITY_AH,
        "cell_count": CELL_COUNT,
    }

    tmp = STATE_FILE.with_suffix(".tmp")
    with tmp.open("w") as f:
        json.dump(data, f, indent=2)
    tmp.replace(STATE_FILE)


def handle_shutdown_signal(signum, _frame):
    global STOP_REQUESTED
    if not STOP_REQUESTED:
        print(f"Received signal {signum}, shutting down...")
    STOP_REQUESTED = True


def build_mqtt_client():
    if not MQTT_ENABLED:
        return None

    if mqtt is None:
        raise RuntimeError(
            "MQTT publishing is enabled but paho-mqtt is not installed. "
            "Install it with: python3 -m pip install paho-mqtt"
        )

    client = mqtt.Client(client_id=MQTT_CLIENT_ID, clean_session=True)
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD or "")

    client.will_set(MQTT_AVAILABILITY_TOPIC, payload="offline", qos=MQTT_QOS, retain=True)
    client.reconnect_delay_set(min_delay=1, max_delay=30)

    def on_connect(client, _userdata, _flags, rc):
        if rc == 0:
            print(f"MQTT connected: {MQTT_HOST}:{MQTT_PORT}")
            client.publish(MQTT_AVAILABILITY_TOPIC, payload="online", qos=MQTT_QOS, retain=True)
        else:
            print(f"MQTT connection failed with rc={rc}")

    def on_disconnect(_client, _userdata, rc):
        if rc != 0:
            print(f"MQTT disconnected unexpectedly (rc={rc})")

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.connect_async(MQTT_HOST, MQTT_PORT, MQTT_KEEPALIVE_S)
    client.loop_start()
    return client


def publish_mqtt_state(client, payload):
    if client is None:
        return
    client.publish(
        MQTT_STATE_TOPIC,
        payload=json.dumps(payload, separators=(",", ":"), sort_keys=True),
        qos=MQTT_QOS,
        retain=True,
    )


def publish_mqtt_offline(client):
    if client is None:
        return
    info = client.publish(
        MQTT_AVAILABILITY_TOPIC,
        payload="offline",
        qos=MQTT_QOS,
        retain=True,
    )
    try:
        info.wait_for_publish(timeout=1.0)
    except TypeError:
        info.wait_for_publish()


def read_ina260(ina):
    voltage_v = float(ina.voltage)
    raw_current_a = float(ina.current) / 1000.0
    power_w = float(ina.power) / 1000.0

    discharge_current_a = DISCHARGE_CURRENT_SIGN * raw_current_a

    return voltage_v, raw_current_a, discharge_current_a, power_w


def initialize_soc_from_voltage(ina, duration_s=5.0):
    """
    On startup, average a few seconds of readings and estimate initial SOC.
    """
    readings = []
    start = time.time()

    while time.time() - start < duration_s and not STOP_REQUESTED:
        voltage_v, raw_current_a, discharge_current_a, power_w = read_ina260(ina)
        readings.append((voltage_v, raw_current_a, discharge_current_a, power_w))
        time.sleep(0.5)

    if not readings:
        voltage_v, raw_current_a, discharge_current_a, power_w = read_ina260(ina)
        readings.append((voltage_v, raw_current_a, discharge_current_a, power_w))

    avg_voltage = sum(r[0] for r in readings) / len(readings)
    avg_raw_current = sum(r[1] for r in readings) / len(readings)
    avg_discharge_current = sum(r[2] for r in readings) / len(readings)
    avg_power = sum(r[3] for r in readings) / len(readings)

    voltage_soc, ocv_pack_v, ocv_cell_v = estimate_voltage_soc(
        avg_voltage,
        avg_discharge_current,
    )

    remaining_ah = BATTERY_CAPACITY_AH * voltage_soc / 100.0

    return {
        "soc_percent": voltage_soc,
        "remaining_ah": remaining_ah,
        "voltage_v": avg_voltage,
        "raw_current_a": avg_raw_current,
        "discharge_current_a": avg_discharge_current,
        "power_w": avg_power,
        "ocv_pack_v": ocv_pack_v,
        "ocv_cell_v": ocv_cell_v,
    }


def main():
    signal.signal(signal.SIGINT, handle_shutdown_signal)
    signal.signal(signal.SIGTERM, handle_shutdown_signal)

    mqtt_client = build_mqtt_client()
    i2c = board.I2C()
    ina = adafruit_ina260.INA260(i2c)

    state = load_state()

    startup_estimate = initialize_soc_from_voltage(ina)

    if state is None:
        soc_percent = startup_estimate["soc_percent"]
        remaining_ah = startup_estimate["remaining_ah"]
        init_source = "voltage_only_no_saved_state"
    else:
        saved_remaining_ah = float(state.get("remaining_ah", startup_estimate["remaining_ah"]))
        saved_soc = 100.0 * saved_remaining_ah / BATTERY_CAPACITY_AH

        # On startup, blend saved coulomb state with fresh voltage estimate.
        # This handles reboot drift, but does not completely trust voltage under load.
        voltage_soc = startup_estimate["soc_percent"]

        soc_percent = 0.70 * saved_soc + 0.30 * voltage_soc
        soc_percent = clamp(soc_percent, MIN_SOC_PERCENT, MAX_SOC_PERCENT)
        remaining_ah = BATTERY_CAPACITY_AH * soc_percent / 100.0
        init_source = "saved_state_blended_with_voltage"

    print("Battery SOC monitor started")
    print(f"Init source: {init_source}")
    print(f"Initial SOC: {soc_percent:.1f}%")
    print(f"Initial remaining Ah: {remaining_ah:.2f}Ah")
    print(f"Cell count: {CELL_COUNT}S")
    if MQTT_ENABLED:
        print(
            f"MQTT publish: {MQTT_HOST}:{MQTT_PORT} "
            f"topic={MQTT_STATE_TOPIC}"
        )
    print()

    last_t = time.time()
    last_save_t = last_t
    last_voltage_v = None
    last_payload = None

    # Simple low-pass filtered values
    filtered_voltage_v = startup_estimate["voltage_v"]
    filtered_current_a = startup_estimate["discharge_current_a"]

    try:
        while not STOP_REQUESTED:
            loop_start = time.time()

            voltage_v, raw_current_a, discharge_current_a, power_w = read_ina260(ina)

            # Low-pass filtering
            filtered_voltage_v = 0.90 * filtered_voltage_v + 0.10 * voltage_v
            filtered_current_a = 0.85 * filtered_current_a + 0.15 * discharge_current_a

            now = time.time()
            dt_h = (now - last_t) / 3600.0
            last_t = now

            # Coulomb counting.
            # Positive discharge_current_a decreases remaining Ah.
            # Negative discharge_current_a means charging/regeneration, if present.
            remaining_ah -= filtered_current_a * dt_h
            remaining_ah = clamp(remaining_ah, 0.0, BATTERY_CAPACITY_AH)

            coulomb_soc = 100.0 * remaining_ah / BATTERY_CAPACITY_AH

            voltage_soc, ocv_pack_v, ocv_cell_v = estimate_voltage_soc(
                filtered_voltage_v,
                filtered_current_a,
            )

            # Decide whether voltage estimate is trustworthy enough to correct drift.
            voltage_stable = False
            if last_voltage_v is not None:
                voltage_delta = abs(filtered_voltage_v - last_voltage_v)
                voltage_stable = voltage_delta < STABLE_VOLTAGE_DELTA_THRESHOLD_V
            else:
                voltage_delta = 0.0

            low_current = abs(filtered_current_a) < REST_CURRENT_THRESHOLD_A

            if low_current and voltage_stable:
                # Slowly nudge coulomb count toward voltage estimate.
                corrected_soc = (
                    (1.0 - VOLTAGE_CORRECTION_GAIN) * coulomb_soc
                    + VOLTAGE_CORRECTION_GAIN * voltage_soc
                )
                remaining_ah = BATTERY_CAPACITY_AH * corrected_soc / 100.0
                soc_percent = corrected_soc
                correction_active = True
            else:
                soc_percent = coulomb_soc
                correction_active = False

            # Hard endpoint clamps.
            # These protect against obviously impossible estimates.
            cell_v = filtered_voltage_v / CELL_COUNT

            if cell_v >= 4.18 and abs(filtered_current_a) < REST_CURRENT_THRESHOLD_A:
                soc_percent = max(soc_percent, 98.0)
                remaining_ah = BATTERY_CAPACITY_AH * soc_percent / 100.0

            if cell_v <= 3.30:
                soc_percent = min(soc_percent, 2.0)
                remaining_ah = BATTERY_CAPACITY_AH * soc_percent / 100.0

            soc_percent = clamp(soc_percent, MIN_SOC_PERCENT, MAX_SOC_PERCENT)
            remaining_ah = BATTERY_CAPACITY_AH * soc_percent / 100.0

            last_voltage_v = filtered_voltage_v

            display_current_a = -filtered_current_a
            display_power_w = filtered_voltage_v * display_current_a
            warn = soc_percent <= WARN_SOC_PERCENT
            critical = soc_percent <= CRITICAL_SOC_PERCENT

            last_payload = {
                "schema": PAYLOAD_SCHEMA,
                "hostname": socket.gethostname(),
                "updated_at_ms": int(now * 1000.0),
                "stale_after_ms": STALE_AFTER_MS,
                "soc_percent": round(soc_percent, 2),
                "remaining_ah": round(remaining_ah, 3),
                "capacity_ah": BATTERY_CAPACITY_AH,
                "voltage_v": round(filtered_voltage_v, 3),
                "current_a": round(display_current_a, 3),
                "power_w": round(display_power_w, 3),
                "raw_current_a": round(raw_current_a, 3),
                "discharge_current_a": round(filtered_current_a, 3),
                "voltage_soc_percent": round(voltage_soc, 2),
                "ocv_pack_v": round(ocv_pack_v, 3),
                "ocv_cell_v": round(ocv_cell_v, 3),
                "correction_active": correction_active,
                "voltage_stable": voltage_stable,
                "resting": low_current,
                "warn": warn,
                "critical": critical,
                "init_source": init_source,
            }

            publish_mqtt_state(mqtt_client, last_payload)

            print(
                f"SOC={soc_percent:5.1f}% | "
                f"V={filtered_voltage_v:6.2f}V | "
                f"I_discharge={filtered_current_a:6.2f}A | "
                f"P={power_w:7.1f}W | "
                f"V_SOC={voltage_soc:5.1f}% | "
                f"OCV/cell={ocv_cell_v:.3f}V | "
                f"Ah={remaining_ah:5.2f}/{BATTERY_CAPACITY_AH:.2f} | "
                f"Vcorr={'ON' if correction_active else 'off'}"
            )

            if now - last_save_t >= STATE_SAVE_PERIOD_S:
                save_state(
                    soc_percent=soc_percent,
                    remaining_ah=remaining_ah,
                    voltage_v=filtered_voltage_v,
                    current_a=filtered_current_a,
                )
                last_save_t = now

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, SAMPLE_PERIOD_S - elapsed))
    finally:
        save_state(
            soc_percent=soc_percent,
            remaining_ah=remaining_ah,
            voltage_v=filtered_voltage_v,
            current_a=filtered_current_a,
        )
        if last_payload is not None:
            publish_mqtt_state(mqtt_client, last_payload)
        if mqtt_client is not None:
            publish_mqtt_offline(mqtt_client)
            mqtt_client.loop_stop()
            mqtt_client.disconnect()


if __name__ == "__main__":
    main()
