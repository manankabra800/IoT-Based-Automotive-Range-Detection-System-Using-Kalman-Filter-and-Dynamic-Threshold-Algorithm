"""
IoT-Based Automotive Range Detection System
REAL MQTT DEMO — Phase 1 Final Review

This script publishes ACTUAL live MQTT messages to HiveMQ
public broker — demonstrating real IoT connectivity.

HOW TO RUN:
1. pip install paho-mqtt numpy
2. python mqtt_live_demo.py
3. Open https://www.hivemq.com/demos/websocket-client/
4. Connect to: broker.hivemq.com port 8000
5. Subscribe to topic: iot/vehicle/range
6. Watch live JSON messages arriving!

Authors: Ajinkya Bhagwat, Manan Kabra, Rishita Modi
NMIT Bengaluru | VTU | 2026-27
"""

import paho.mqtt.client as mqtt
import json
import time
import math
import random
import numpy as np
from datetime import datetime

# ── HiveMQ Public Broker ─────────────────────────────────────────────
BROKER   = "broker.hivemq.com"
PORT     = 1883
TOPIC    = "iot/vehicle/range"
CLIENT_ID = "NMIT_IoT_RangeDetection_ESP32"

# ── Kalman Filter State ──────────────────────────────────────────────
kf_x = 300.0
kf_p = 1.0
KF_Q = 0.01
KF_R = 0.50

def kalman_update(z, R=KF_R):
    global kf_x, kf_p
    kf_p = kf_p + KF_Q
    K    = kf_p / (kf_p + R)
    kf_x = kf_x + K * (z - kf_x)
    kf_p = (1 - K) * kf_p
    return kf_x

def safe_distance(speed_kmh):
    v = speed_kmh / 3.6
    return max(20, (v * 0.7 + v * v / 14.0) * 100)

# ── Simulation State ─────────────────────────────────────────────────
speed_kmh  = 0.0
speed_dir  = 1.0
true_dist  = 400.0
dist_dir   = -1.0
msg_count  = 0

def simulate_tick():
    global speed_kmh, speed_dir, true_dist, dist_dir, msg_count

    # Update speed
    speed_kmh += speed_dir * 0.8
    if speed_kmh >= 80: speed_dir = -1
    if speed_kmh <= 0:  speed_dir =  1

    # Update true distance
    true_dist += dist_dir * (0.5 + speed_kmh * 0.03)
    if true_dist < 25:  dist_dir =  1
    if true_dist > 550: dist_dir = -1

    # Sensor readings with noise
    noise1 = random.gauss(0, 8)
    noise2 = random.gauss(0, 2.5)
    # 5% chance of spike on HC-SR04
    if random.random() < 0.05:
        noise1 += random.choice([-1, 1]) * random.uniform(25, 55)

    s1 = max(2, min(600, true_dist + noise1))
    s2 = max(2, min(600, true_dist + noise2))

    # Kalman fusion — dual pass
    kalman_update(s1, KF_R)
    fused = kalman_update((s1 + s2) / 2, KF_R * 0.25)

    # Safe distance and alert
    safe  = safe_distance(speed_kmh)
    if   fused < safe * 0.5: alert = 2  # DANGER
    elif fused < safe:        alert = 1  # WARNING
    else:                     alert = 0  # SAFE

    msg_count += 1

    payload = {
        "msg_id":       msg_count,
        "distance_cm":  round(fused, 1),
        "speed_kmh":    round(speed_kmh, 1),
        "safe_dist_cm": round(safe, 1),
        "sensor1_cm":   round(s1, 1),
        "sensor2_cm":   round(s2, 1),
        "alert_level":  alert,
        "alert_status": ["SAFE", "WARNING", "DANGER"][alert],
        "timestamp":    datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z",
        "device_id":    "ESP32_NMIT_001",
        "topic":        TOPIC
    }
    return payload

# ── MQTT Callbacks ───────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("\n" + "="*60)
        print("  ✅  CONNECTED TO HiveMQ BROKER SUCCESSFULLY!")
        print("="*60)
        print(f"  Broker  : {BROKER}:{PORT}")
        print(f"  Topic   : {TOPIC}")
        print(f"  Client  : {CLIENT_ID}")
        print("="*60)
        print("\n  📡  Publishing live MQTT messages at 5 Hz...")
        print("  🌐  View at: https://www.hivemq.com/demos/websocket-client/")
        print(f"  📋  Subscribe to topic: {TOPIC}")
        print("\n  Press Ctrl+C to stop\n")
        print("-"*60)
    else:
        print(f"  ❌  Connection failed with code {rc}")

def on_disconnect(client, userdata, rc):
    print(f"\n  Disconnected from broker (code: {rc})")

def on_publish(client, userdata, mid):
    pass  # Silent publish confirmation

# ── Main ─────────────────────────────────────────────────────────────
def main():
    print("\n" + "="*60)
    print("  IoT AUTOMOTIVE RANGE DETECTION — MQTT LIVE DEMO")
    print("  NMIT Bengaluru | VTU | Academic Year 2026-27")
    print("="*60)
    print(f"\n  Connecting to {BROKER}:{PORT}...")

    client = mqtt.Client(client_id=CLIENT_ID, clean_session=True)
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_publish    = on_publish

    try:
        client.connect(BROKER, PORT, keepalive=60)
        client.loop_start()

        # Wait for connection
        time.sleep(2)

        while True:
            payload = simulate_tick()
            json_str = json.dumps(payload, indent=None)

            result = client.publish(TOPIC, json_str, qos=0)

            # Print to terminal
            status = ["🟢 SAFE   ", "🟡 WARNING", "🔴 DANGER "][payload["alert_level"]]
            print(f"  [{payload['msg_id']:04d}] {status} | dist={payload['distance_cm']:6.1f}cm | "
                  f"spd={payload['speed_kmh']:5.1f}km/h | "
                  f"safe={payload['safe_dist_cm']:6.1f}cm | "
                  f"{payload['timestamp'][11:23]}")

            time.sleep(0.2)  # 5 Hz

    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print(f"  Demo stopped after {msg_count} messages published")
        print(f"  All messages sent to: {BROKER}/{TOPIC}")
        print("="*60)
        client.loop_stop()
        client.disconnect()

    except Exception as e:
        print(f"\n  ❌  Error: {e}")
        print("  Check your internet connection and try again")

if __name__ == "__main__":
    main()
