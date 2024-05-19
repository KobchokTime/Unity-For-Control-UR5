using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using M2MqttUnity;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;
using UnityEngine.UI;

public class MQTTColorController : MonoBehaviour
{
    [Header("MQTT Receiver")]
    public mqttReceiver mqttReceiverScript;

    [Header("RawImage Components")]
    public RawImage[] rawImages;

    // Default color for RawImage components
    private Color defaultColor = Color.red;

    // Map of colors corresponding to their names
    private readonly Dictionary<string, Color> colorMap = new Dictionary<string, Color>
    {
        { "green", new Color(0.082f, 0.855f, 0.063f, 1.0f) } // 15DA10
        // Add more colors if needed
    };

    void Start()
    {
        // Set default color for all RawImage components
        foreach (RawImage rawImage in rawImages)
        {
            rawImage.color = defaultColor;
        }

        // Ensure MQTT receiver script is assigned
        if (mqttReceiverScript == null)
        {
            // Attempt to find mqttReceiver script in the scene
            mqttReceiverScript = FindObjectOfType<mqttReceiver>();
            if (mqttReceiverScript == null)
            {
                Debug.LogError("MQTT receiver script is not assigned and cannot be found in the scene.");
            }
        }
        else
        {
            // Subscribe to MQTT message event
            mqttReceiverScript.OnMessageArrived += HandleMessageReceived;
        }
    }

    void OnDestroy()
    {
        // Unsubscribe from MQTT message event
        if (mqttReceiverScript != null)
        {
            mqttReceiverScript.OnMessageArrived -= HandleMessageReceived;
        }
    }

    // Method to handle message received for each RawImage component
    public void HandleMessageReceived(string newMsg)
    {
        // Split the message into color values
        string[] colorValues = newMsg.Split(',');

        // Check if the number of color values matches the number of RawImage components
        if (colorValues.Length != rawImages.Length)
        {
            Debug.LogError("Number of color values received does not match the number of RawImage components.");
            return;
        }

        // Iterate through each RawImage component and corresponding color value
        for (int i = 0; i < rawImages.Length; i++)
        {
            // Parse the color value
            int colorValue;
            if (int.TryParse(colorValues[i], out colorValue))
            {
                // Check if the color value indicates a color change (1)
                if (colorValue == 1)
                {
                    // Change the color of the corresponding RawImage component to green
                    rawImages[i].color = colorMap["green"];
                }
                else
                {
                    // Reset to default color
                    rawImages[i].color = defaultColor;
                }
            }
            else
            {
                Debug.LogError("Invalid color value received: " + colorValues[i]);
            }
        }
    }
}