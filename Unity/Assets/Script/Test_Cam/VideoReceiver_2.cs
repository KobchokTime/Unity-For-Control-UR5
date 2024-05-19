using UnityEngine;
using UnityEngine.UI;
using System.Net.Sockets;
using System;
using System.Threading;

public class VideoReceiver_2 : MonoBehaviour
{
    public RawImage rawImage;
    private TcpClient client;
    private NetworkStream stream;
    private Texture2D texture;
    private byte[] receivedData;

    void Start()
    {
        // Ensure UnityMainThreadDispatcher is initialized
        if (UnityMainThreadDispatcher.Instance() == null)
        {
            Debug.LogError("UnityMainThreadDispatcher is not initialized.");
            return;
        }

        // Initialize texture on the main thread
        texture = new Texture2D(2, 2);
        rawImage.texture = texture;

        // Start the client thread
        Thread clientThread = new Thread(StartClient);
        clientThread.IsBackground = true;
        clientThread.Start();
    }

    void StartClient()
    {
        try
        {
            client = new TcpClient("127.0.0.1", 9090);  // Replace with the actual IP address of PC2
            stream = client.GetStream();
            receivedData = new byte[1024 * 1024];  // Adjust buffer size as needed

            while (true)
            {
                // Read the frame size
                byte[] sizeInfo = new byte[4];
                stream.Read(sizeInfo, 0, 4);
                int frameSize = BitConverter.ToInt32(sizeInfo, 0);

                // Read the frame data
                int totalRead = 0;
                while (totalRead < frameSize)
                {
                    int read = stream.Read(receivedData, totalRead, frameSize - totalRead);
                    if (read == 0)
                    {
                        break;
                    }
                    totalRead += read;
                }

                // Update the texture on the main thread
                UnityMainThreadDispatcher.Instance().Enqueue(() =>
                {
                    texture.LoadImage(receivedData);
                    rawImage.texture = texture;
                });
            }
        }
        catch (Exception ex)
        {
            Debug.LogError("Error: " + ex.Message);
        }
        finally
        {
            stream.Close();
            client.Close();
        }
    }
}
