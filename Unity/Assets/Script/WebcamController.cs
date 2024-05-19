using UnityEngine;
using UnityEngine.UI;

public class WebcamController : MonoBehaviour
{
    public RawImage rawImage;
    public string externalWebcamName = "Name of your external webcam";

    void Start()
    {
        // Get the list of available devices (webcams)
        WebCamDevice[] devices = WebCamTexture.devices;

        // Check if there's at least one webcam available
        if (devices.Length > 0)
        {
            // Find the external webcam device by name
            WebCamDevice externalWebcam = System.Array.Find(devices, device => device.name == externalWebcamName);

            // Check if the external webcam is found
            if (externalWebcam.name != null)
            {
                // Create a WebCamTexture using the external webcam
                WebCamTexture webcamTexture = new WebCamTexture(externalWebcam.name);

                // Assign the webcam texture to the RawImage component
                rawImage.texture = webcamTexture;

                // Start capturing from the webcam
                webcamTexture.Play();
            }
            else
            {
                Debug.LogError("External webcam not found!");
            }
        }
        else
        {
            Debug.LogError("No webcam found!");
        }
    }
}
