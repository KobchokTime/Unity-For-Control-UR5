using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Testmqtt : MonoBehaviour
{
    public mqttReceiver mqttReceiverInstance; // Reference to the mqttReceiver object

    void Start()
    {
        // Ensure that mqttReceiverInstance is assigned properly
        if (mqttReceiverInstance == null)
        {
            Debug.LogError("mqttReceiverInstance is not assigned!");
            return;
        }

        // Call the Publish() method of mqttReceiverInstance
        mqttReceiverInstance.Publish();
    }
}