using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using UnityEngine.UI;

public class VideoReceiver : MonoBehaviour
{
    public RawImage rawImage; // Reference to the RawImage component where you want to display the video
    public mqttReceiver mqttReceiver; // Reference to the MQTT receiver script

    void Start()
    {
        // Subscribe to the event for receiving messages
        mqttReceiver.OnMessageArrived += OnMqttMessageArrived;
    }

    void OnDestroy()
    {
        // Unsubscribe from the event when the object is destroyed
        mqttReceiver.OnMessageArrived -= OnMqttMessageArrived;
    }

    // Method to handle MQTT messages
    void OnMqttMessageArrived(string newMsg)
    {
        // Assuming the MQTT message contains image data in bytes
        byte[] imageData = System.Convert.FromBase64String(newMsg);

        // Convert the image data to a texture
        Texture2D texture = new Texture2D(1, 1);
        texture.LoadImage(imageData);

        // Apply the texture to the RawImage component
        rawImage.texture = texture;
    }
}