using System.Collections;
using UnityEngine;
using UnityEngine.Networking;
using UnityEngine.UI;

public class CameraReceiver : MonoBehaviour
{
    public string serverUrl = "http://10.7.147.10:8000/video_feed"; // Replace PC1_IP_ADDRESS with the actual IP address of PC1
    public RawImage rawImage;

    private Texture2D texture;

    void Start()
    {
        texture = new Texture2D(2, 2); // Initial texture size
        rawImage.texture = texture;
        StartCoroutine(FetchFrames());
    }

    private IEnumerator FetchFrames()
    {
        while (true)
        {
            UnityWebRequest www = UnityWebRequestTexture.GetTexture(serverUrl);
            yield return www.SendWebRequest();

            if (www.result != UnityWebRequest.Result.Success)
            {
                Debug.LogError("Error fetching frame: " + www.error);
            }
            else
            {
                Debug.Log("Frame fetched successfully.");
                Texture2D receivedTexture = DownloadHandlerTexture.GetContent(www);
                if (receivedTexture != null)
                {
                    Debug.Log("Received texture is valid.");
                    rawImage.texture = receivedTexture;
                    rawImage.SetNativeSize(); // Ensure the RawImage adjusts to the texture size
                }
                else
                {
                    Debug.LogError("Received texture is null.");
                }
            }
        }
    }
}