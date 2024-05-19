using UnityEngine;
using UnityEngine.UI;

public class WebcamDisplay : MonoBehaviour
{
    public RawImage rawImage;
    private WebCamTexture[] webcams;
    private int currentCameraIndex = 0;

    void Start()
    {
        // Check if there's any webcam available
        if (WebCamTexture.devices.Length == 0)
        {
            Debug.LogError("No webcam found!");
            return;
        }

        // Initialize array to store all available webcams
        webcams = new WebCamTexture[WebCamTexture.devices.Length];

        // Start the default webcam
        SwitchCamera(currentCameraIndex);
    }

    public void SwapCamera()
    {
        // Stop the current webcam
        webcams[currentCameraIndex].Stop();

        // Move to the next camera
        currentCameraIndex = (currentCameraIndex + 1) % webcams.Length;

        // Start the new webcam
        SwitchCamera(currentCameraIndex);
    }

    private void SwitchCamera(int index)
    {
        // Access the selected webcam
        webcams[index] = new WebCamTexture(WebCamTexture.devices[index].name);
        rawImage.texture = webcams[index];

        // Start the webcam
        webcams[index].Play();
    }
}
