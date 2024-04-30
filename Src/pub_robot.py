import paho.mqtt.client as mqtt
import time

# Define MQTT broker address and topic
broker_address = "192.168.137.1"  # Replace with your broker address
topic = "robot_commands"

# Create an MQTT client and connect to the broker
client = mqtt.Client()
client.connect(broker_address)

# Define a function to publish a robot command
def publish_command(robot_id, command, parameters):
    command_str = ",".join([robot_id, command] + list(map(str, parameters)))
    client.publish(topic, command_str)

publish_command("20", "movej", [-0.8, -0.07235, 0.18361, 2.495, -0.867, -2.423, 0.5, 0.5])
publish_command("10", "movej", [0.8, -0.04809, 0.12294, 2.431, -1.084, 2.588, 0.5, 0.5])
time.sleep(2)
publish_command("20", "movej", [-0.70365, -0.07235, 0.18361, 2.495, -0.867, -2.423, 0.5, 0.5])
publish_command("10", "movej", [0.73875, -0.04809, 0.12294, 2.431, -1.084, 2.588, 0.5, 0.5])
time.sleep(1)