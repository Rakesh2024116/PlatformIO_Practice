import paho.mqtt.client as mqtt
import time
import json
import base64

# MQTT settings
MQTT_BROKER = "3.17.163.180"
MQTT_PORT = 1883
MQTT_USERNAME = "moambulance"
MQTT_PASSWORD = "P@$sw0rd2001"
MQTT_TOPIC = "amb/ota"

# Firmware file path
FIRMWARE_FILE = "C:/Users/Asus/Desktop/Clinohealth/program/ESP32_Basics/mac_add_esp32/build/esp32.esp32.esp32/mac_add_esp32.ino.bin"

# Chunk size (1024 bytes)
CHUNK_SIZE = 1024

def split_firmware(filename, chunk_size):
    with open(filename, 'rb') as f:
        while True:
            chunk = f.read(chunk_size)
            if not chunk:
                break
            yield chunk

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")

client = mqtt.Client()
client.on_connect = on_connect
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Read and split the firmware
chunks = list(split_firmware(FIRMWARE_FILE, CHUNK_SIZE))
total_chunks = len(chunks)

# Send initialization message
init_message = json.dumps({"type": "init", "total_chunks": total_chunks, "chunk_size": CHUNK_SIZE})
client.publish(MQTT_TOPIC, init_message)

# Send chunks
for i, chunk in enumerate(chunks):
    chunk_data = json.dumps({"type": "chunk", "index": i, "data": base64.b64encode(chunk).decode('utf-8')})
    client.publish(MQTT_TOPIC, chunk_data)
    print(f"Published chunk {i+1}/{total_chunks}")
    time.sleep(0.1)  # Adjust delay as needed

print("Firmware update complete")
client.disconnect()
