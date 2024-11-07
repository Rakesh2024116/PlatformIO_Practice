import paho.mqtt.client as mqtt
import time
import json

# MQTT settings
MQTT_BROKER = "3.17.163.180"        # IP address of the MQTT broker
MQTT_PORT = 1883                    # Port of the MQTT broker
MQTT_USERNAME = "moambulance"       # Username of the MQTT broker
MQTT_PASSWORD = "P@$sw0rd2001"      # Password of the MQTT broker
MQTT_TOPIC_INIT = "amb/ota/init"
MQTT_TOPIC_CHUNK = "amb/ota/chunk"

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
init_message = json.dumps({"total_chunks": total_chunks, "chunk_size": CHUNK_SIZE})
client.publish(MQTT_TOPIC_INIT, init_message)

# Send chunks
for i, chunk in enumerate(chunks):
    chunk_data = i.to_bytes(2, 'little') + chunk
    client.publish(MQTT_TOPIC_CHUNK, chunk_data)
    print(f"Published chunk {i+1}/{total_chunks}")
    time.sleep(0.1)  # Adjust delay as needed

print("Firmware update complete")
client.disconnect()
