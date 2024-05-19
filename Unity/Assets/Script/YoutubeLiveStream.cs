using UnityEngine;
using UnityEngine.UI;
using System.Diagnostics;
using UnityEngine.Video;

public class YouTubeLiveStream : MonoBehaviour
{
    public RawImage rawImage;
    public string youtubeVideoID; // YouTube video ID of the live stream

    private Process youtubeDLProcess;
    private VideoPlayer videoPlayer;
    private RenderTexture renderTexture;

    void Start()
    {
        // Use YouTube-DL to get the direct video stream URL
        string youtubeDLPath = "path/to/youtube-dl"; // Path to youtube-dl executable
        string arguments = $"-g https://www.youtube.com/watch?v={youtubeVideoID}";
        ProcessStartInfo startInfo = new ProcessStartInfo(youtubeDLPath, arguments);
        startInfo.RedirectStandardOutput = true;
        startInfo.UseShellExecute = false;

        youtubeDLProcess = new Process();
        youtubeDLProcess.StartInfo = startInfo;
        youtubeDLProcess.OutputDataReceived += (sender, e) =>
        {
            if (!string.IsNullOrEmpty(e.Data))
            {
                // When youtube-dl retrieves the direct video URL, play it
                PlayVideo(e.Data);
            }
        };

        // Start the youtube-dl process
        youtubeDLProcess.Start();
        youtubeDLProcess.BeginOutputReadLine();
    }

    void PlayVideo(string videoURL)
    {
        // Create a new video player
        videoPlayer = gameObject.AddComponent<VideoPlayer>();

        // Create a new render texture
        renderTexture = new RenderTexture(1920, 1080, 24);
        renderTexture.Create();

        // Set the render texture as target texture for the video player
        videoPlayer.targetTexture = renderTexture;

        // Set the raw image's texture to the render texture
        rawImage.texture = renderTexture;

        // Set the video player's URL to stream the video
        videoPlayer.url = videoURL;

        // Play the video
        videoPlayer.Play();
    }

    void OnDestroy()
    {
        // Close the youtube-dl process when the script is destroyed
        if (youtubeDLProcess != null && !youtubeDLProcess.HasExited)
        {
            youtubeDLProcess.Kill();
        }
    }
}
