using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Networking;
using System.Collections;
using UnityEngine.InputSystem;

public class WebCamCapture : MonoBehaviour
{
    public RawImage rawImage;
    public string url = "http://10.7.147.10:8000/video_feed"; // URL of the website you want to capture

    IEnumerator Start()
    {
        UnityWebRequest www = UnityWebRequestTexture.GetTexture(url);
        yield return www.SendWebRequest();

        if (www.result == UnityWebRequest.Result.ConnectionError || www.result == UnityWebRequest.Result.ProtocolError)
        {
            Debug.LogError("Error: " + www.error);
        }
        else
        {
            Texture2D texture = DownloadHandlerTexture.GetContent(www);
            if (texture != null)
            {
                Debug.Log("Texture loaded successfully");
                rawImage.texture = texture;
            }
            else
            {
                Debug.LogError("Failed to load texture");
            }
        }
    }
}