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
FIRMWARE_FILE = "C:/Users/Asus/Desktop/Practice_Prog/platformio/Blink_code/.pio/build/esp32dev/firmware.bin"

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

# Read the first chunk of the firmware
chunk_generator = split_firmware(FIRMWARE_FILE, CHUNK_SIZE)
first_chunk = next(chunk_generator)

# Determine total number of chunks
total_chunks = sum(1 for _ in chunk_generator) + 1

# Send initialization message
# init_message = json.dumps({"total_chunks": total_chunks, "chunk_size": CHUNK_SIZE})
# client.publish(MQTT_TOPIC_INIT, init_message)

# Send the first chunk
chunk_data = b'\x00\x00' + first_chunk  # Chunk number 0
client.publish(MQTT_TOPIC_CHUNK, chunk_data)
print("Published the first chunk")

print("Firmware update initiation complete")
client.disconnect()
