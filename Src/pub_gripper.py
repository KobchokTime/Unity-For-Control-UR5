import paho.mqtt.client as mqtt
import time

# Define MQTT broker address and topic
broker_address = "192.168.137.1" # Replace with your broker address
topic = "gripper_commands"

# Create an MQTT client and connect to the broker
client = mqtt.Client()
client.connect(broker_address)

# Define a function to publish a gripper command
def publish_command(command):
    client.publish(topic, command)


publish_command("close1")
publish_command("open2 140") # close
time.sleep(3)
publish_command("open1")
publish_command("open2 40") # open
time.sleep(1)
# publish_command("close2") close 100%

