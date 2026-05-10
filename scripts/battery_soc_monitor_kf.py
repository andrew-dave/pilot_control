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

# Your "24V" Li-ion battery charges to 29.4V, so it is a 7S Li-ion pack.
# 7S Li-ion: full = 29.4V, nominal = 25.9V, absolute empty is usually near 21.0V.
CELL_COUNT = 7

# Pack-level voltage model used as the Kalman voltage measurement.
# PACK_EMPTY_V should be treated as "operationally empty", not necessarily BMS cutoff.
# Use 21.0V to match a simple full-to-empty pack gauge.
# Use 22.4V for a more conservative robot runtime estimate.
PACK_FULL_V = 29.4
PACK_EMPTY_V = 21.0

# If positive INA260 current means battery is discharging, leave +1.
# If your current is negative during discharge, set to -1.
DISCHARGE_CURRENT_SIGN = 1

# Approximate effective pack + wiring + connector + BMS resistance in ohms.
# Used to estimate open-circuit/resting voltage under load:
# V_ocv_est = V_measured + I_discharge * R_pack
# Tune from logs: R ~= (V_rest - V_load) / I_load.
PACK_INTERNAL_RESISTANCE_OHM = 0.05

# Sampling and persistence
SAMPLE_PERIOD_S = 1.0
STATE_SAVE_PERIOD_S = 15.0
STATE_FILE = Path("/var/tmp/battery_soc_state.json")
ESTIMATOR_VERSION = "kf-pack-linear-v1"

# Kalman filter tuning.
# State x is SOC fraction: 0.0 to 1.0.
# P is variance of SOC fraction estimate.
# R is voltage-derived SOC measurement variance.
# Q is process variance added to the coulomb-count prediction.
INITIAL_KALMAN_P_NO_STATE = 0.04       # about 20% std dev
INITIAL_KALMAN_P_WITH_STATE = 0.01     # about 10% std dev
KALMAN_Q_BASE = 1e-7
KALMAN_Q_CURRENT_SCALE = 2e-7
KALMAN_R_RESTING = 0.0025              # about 5% std dev
KALMAN_R_LIGHT_LOAD = 0.01             # about 10% std dev
KALMAN_R_MODERATE_LOAD = 0.04          # about 20% std dev
KALMAN_R_HEAVY_LOAD = 0.16             # about 40% std dev

# Measurement-quality thresholds
REST_CURRENT_THRESHOLD_A = 1.0
LIGHT_CURRENT_THRESHOLD_A = 4.0
MODERATE_CURRENT_THRESHOLD_A = 10.0
STABLE_VOLTAGE_DELTA_THRESHOLD_V = 0.03
LIGHT_VOLTAGE_DELTA_THRESHOLD_V = 0.08
MODERATE_VOLTAGE_DELTA_THRESHOLD_V = 0.15

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


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def estimate_voltage_soc(pack_voltage_v, discharge_current_a):
    """
    Estimate SOC from current-compensated pack voltage.

    This intentionally uses a pack-level linear model instead of the prior
    pessimistic per-cell OCV table. For your 7S pack:
        PACK_FULL_V  = 29.4V -> 100%
        PACK_EMPTY_V = 21.0V -> 0%

    The voltage measurement is compensated for load sag:
        V_ocv_est = V_measured + I_discharge * R_pack

    This is not treated as ground truth. It is the Kalman measurement z.
    The Kalman filter trusts it strongly only when current is low/stable.
    """
    discharge_current_a = max(0.0, discharge_current_a)
    estimated_ocv_pack_v = pack_voltage_v + discharge_current_a * PACK_INTERNAL_RESISTANCE_OHM
    estimated_ocv_cell_v = estimated_ocv_pack_v / CELL_COUNT

    denominator = PACK_FULL_V - PACK_EMPTY_V
    if denominator <= 0:
        raise ValueError("PACK_FULL_V must be greater than PACK_EMPTY_V")

    soc_fraction = (estimated_ocv_pack_v - PACK_EMPTY_V) / denominator
    soc_fraction = clamp(soc_fraction, MIN_SOC_PERCENT / 100.0, MAX_SOC_PERCENT / 100.0)
    soc_percent = 100.0 * soc_fraction

    return soc_percent, estimated_ocv_pack_v, estimated_ocv_cell_v


def voltage_measurement_variance(current_a, voltage_delta_v):
    """
    Dynamic R for the Kalman update.

    Lower R = trust voltage-derived SOC more.
    Higher R = trust voltage-derived SOC less.

    Voltage is most trustworthy when current is low and voltage is stable.
    It is least trustworthy during motor transients, acceleration, stalls,
    and other high-load events.
    """
    current_a = abs(current_a)
    voltage_delta_v = abs(voltage_delta_v)

    if current_a < REST_CURRENT_THRESHOLD_A and voltage_delta_v < STABLE_VOLTAGE_DELTA_THRESHOLD_V:
        return KALMAN_R_RESTING
    if current_a < LIGHT_CURRENT_THRESHOLD_A and voltage_delta_v < LIGHT_VOLTAGE_DELTA_THRESHOLD_V:
        return KALMAN_R_LIGHT_LOAD
    if current_a < MODERATE_CURRENT_THRESHOLD_A and voltage_delta_v < MODERATE_VOLTAGE_DELTA_THRESHOLD_V:
        return KALMAN_R_MODERATE_LOAD
    return KALMAN_R_HEAVY_LOAD


def is_state_valid(state):
    if state is None:
        return False

    try:
        return (
            state.get("estimator_version") == ESTIMATOR_VERSION
            and int(state.get("cell_count", -1)) == CELL_COUNT
            and abs(float(state.get("capacity_ah", -1.0)) - BATTERY_CAPACITY_AH) < 1e-6
            and abs(float(state.get("pack_full_v", -1.0)) - PACK_FULL_V) < 1e-6
            and abs(float(state.get("pack_empty_v", -1.0)) - PACK_EMPTY_V) < 1e-6
        )
    except Exception:
        return False


def load_state():
    if not STATE_FILE.exists():
        return None

    try:
        with STATE_FILE.open("r") as f:
            return json.load(f)
    except Exception:
        return None


def save_state(soc_percent, remaining_ah, voltage_v, current_a, kalman_p=None):
    data = {
        "timestamp": time.time(),
        "estimator_version": ESTIMATOR_VERSION,
        "soc_percent": soc_percent,
        "remaining_ah": remaining_ah,
        "voltage_v": voltage_v,
        "current_a": current_a,
        "capacity_ah": BATTERY_CAPACITY_AH,
        "cell_count": CELL_COUNT,
        "pack_full_v": PACK_FULL_V,
        "pack_empty_v": PACK_EMPTY_V,
        "pack_internal_resistance_ohm": PACK_INTERNAL_RESISTANCE_OHM,
    }
    if kalman_p is not None:
        data["kalman_p"] = kalman_p

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
    state_valid = is_state_valid(state)

    startup_estimate = initialize_soc_from_voltage(ina)
    voltage_soc_percent = startup_estimate["soc_percent"]
    voltage_soc_fraction = voltage_soc_percent / 100.0

    if state_valid:
        saved_remaining_ah = float(state.get("remaining_ah", startup_estimate["remaining_ah"]))
        saved_soc_fraction = clamp(saved_remaining_ah / BATTERY_CAPACITY_AH, 0.0, 1.0)

        # Saved coulomb count is useful, but a fresh voltage estimate prevents
        # obviously stale state from dominating after long downtime or config edits.
        soc_fraction = clamp(0.70 * saved_soc_fraction + 0.30 * voltage_soc_fraction, 0.0, 1.0)
        kalman_p = float(state.get("kalman_p", INITIAL_KALMAN_P_WITH_STATE))
        kalman_p = clamp(kalman_p, 1e-6, 0.25)
        init_source = "valid_saved_state_blended_with_voltage"
    else:
        soc_fraction = voltage_soc_fraction
        kalman_p = INITIAL_KALMAN_P_NO_STATE
        init_source = "voltage_only_no_valid_saved_state"

    soc_percent = 100.0 * soc_fraction
    remaining_ah = BATTERY_CAPACITY_AH * soc_fraction

    print("Battery SOC monitor started")
    print(f"Estimator version: {ESTIMATOR_VERSION}")
    print(f"Init source: {init_source}")
    print(f"Initial SOC: {soc_percent:.1f}%")
    print(f"Initial remaining Ah: {remaining_ah:.2f}Ah")
    print(f"Startup measured pack V: {startup_estimate['voltage_v']:.3f}V")
    print(f"Startup estimated OCV pack V: {startup_estimate['ocv_pack_v']:.3f}V")
    print(f"Startup estimated OCV cell V: {startup_estimate['ocv_cell_v']:.3f}V")
    print(f"Startup voltage SOC: {voltage_soc_percent:.1f}%")
    print(f"Cell count: {CELL_COUNT}S")
    print(f"Voltage model: {PACK_EMPTY_V:.2f}V=0%, {PACK_FULL_V:.2f}V=100%")
    print(f"Initial Kalman P: {kalman_p:.6f}")
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

    # Initialize loop debug variables so finally block is safe even on early exit.
    kalman_gain = 0.0
    voltage_measurement_r = KALMAN_R_HEAVY_LOAD
    voltage_delta = 0.0
    voltage_stable = False
    low_current = False
    soc_pred = soc_fraction
    voltage_soc = voltage_soc_percent
    ocv_pack_v = startup_estimate["ocv_pack_v"]
    ocv_cell_v = startup_estimate["ocv_cell_v"]

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

            if last_voltage_v is not None:
                voltage_delta = abs(filtered_voltage_v - last_voltage_v)
            else:
                voltage_delta = 0.0

            voltage_stable = voltage_delta < STABLE_VOLTAGE_DELTA_THRESHOLD_V
            low_current = abs(filtered_current_a) < REST_CURRENT_THRESHOLD_A

            # -----------------------------
            # Kalman predict step
            # -----------------------------
            # Current integration is the process model:
            # SOC_pred = SOC_previous - I_discharge * dt / capacity
            soc_pred = soc_fraction - (filtered_current_a * dt_h / BATTERY_CAPACITY_AH)
            soc_pred = clamp(soc_pred, 0.0, 1.0)

            # Increase uncertainty with time and current. This represents current
            # measurement noise, capacity uncertainty, and missed loads.
            dt_scale = max(0.1, (dt_h * 3600.0) / max(SAMPLE_PERIOD_S, 1e-6))
            process_q = (KALMAN_Q_BASE + KALMAN_Q_CURRENT_SCALE * abs(filtered_current_a)) * dt_scale
            p_pred = kalman_p + process_q

            # -----------------------------
            # Kalman measurement update
            # -----------------------------
            voltage_soc, ocv_pack_v, ocv_cell_v = estimate_voltage_soc(
                filtered_voltage_v,
                filtered_current_a,
            )
            voltage_soc_fraction = voltage_soc / 100.0

            voltage_measurement_r = voltage_measurement_variance(filtered_current_a, voltage_delta)
            kalman_gain = p_pred / (p_pred + voltage_measurement_r)

            soc_fraction = soc_pred + kalman_gain * (voltage_soc_fraction - soc_pred)
            soc_fraction = clamp(soc_fraction, 0.0, 1.0)
            kalman_p = (1.0 - kalman_gain) * p_pred

            # Endpoint clamps for obvious full/empty cases.
            cell_v = filtered_voltage_v / CELL_COUNT
            if cell_v >= 4.18 and abs(filtered_current_a) < REST_CURRENT_THRESHOLD_A:
                soc_fraction = max(soc_fraction, 0.98)
                kalman_p = min(kalman_p, KALMAN_R_RESTING)

            if filtered_voltage_v <= PACK_EMPTY_V or cell_v <= 3.00:
                soc_fraction = min(soc_fraction, 0.02)
                kalman_p = min(kalman_p, KALMAN_R_RESTING)

            soc_fraction = clamp(soc_fraction, MIN_SOC_PERCENT / 100.0, MAX_SOC_PERCENT / 100.0)
            soc_percent = 100.0 * soc_fraction
            remaining_ah = BATTERY_CAPACITY_AH * soc_fraction

            last_voltage_v = filtered_voltage_v

            # Preserve existing UI sign convention.
            display_current_a = -filtered_current_a
            display_power_w = filtered_voltage_v * display_current_a
            warn = soc_percent <= WARN_SOC_PERCENT
            critical = soc_percent <= CRITICAL_SOC_PERCENT
            correction_active = kalman_gain > 1e-6

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
                "soc_pred_percent": round(100.0 * soc_pred, 2),
                "ocv_pack_v": round(ocv_pack_v, 3),
                "ocv_cell_v": round(ocv_cell_v, 3),
                "kalman_gain": round(kalman_gain, 5),
                "kalman_p": round(kalman_p, 8),
                "voltage_measurement_r": round(voltage_measurement_r, 6),
                "voltage_delta_v": round(voltage_delta, 4),
                "correction_active": correction_active,
                "voltage_stable": voltage_stable,
                "resting": low_current,
                "state_valid": state_valid,
                "estimator_version": ESTIMATOR_VERSION,
                "pack_full_v": PACK_FULL_V,
                "pack_empty_v": PACK_EMPTY_V,
                "pack_internal_resistance_ohm": PACK_INTERNAL_RESISTANCE_OHM,
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
                f"K={kalman_gain:.4f} | "
                f"P={kalman_p:.6f} | "
                f"R={voltage_measurement_r:.4f}"
            )

            if now - last_save_t >= STATE_SAVE_PERIOD_S:
                save_state(
                    soc_percent=soc_percent,
                    remaining_ah=remaining_ah,
                    voltage_v=filtered_voltage_v,
                    current_a=filtered_current_a,
                    kalman_p=kalman_p,
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
            kalman_p=kalman_p,
        )
        if last_payload is not None:
            publish_mqtt_state(mqtt_client, last_payload)
        if mqtt_client is not None:
            publish_mqtt_offline(mqtt_client)
            mqtt_client.loop_stop()
            mqtt_client.disconnect()


if __name__ == "__main__":
    main()
