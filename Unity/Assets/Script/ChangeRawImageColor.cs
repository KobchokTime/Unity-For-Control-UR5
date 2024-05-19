using UnityEngine;
using UnityEngine.UI;

public class ChangeRawImageColor : MonoBehaviour
{
    public RawImage rawImage;
    public string hexColor = "DF2222";

    private void Start()
    {
        // Get the RawImage component if it's not assigned
        if (rawImage == null)
            rawImage = GetComponent<RawImage>();

        // Convert hex color to Color
        Color color = HexToColor(hexColor);

        // Set the initial color of the RawImage
        rawImage.color = color;
    }

    public void OnButtonClick()
    {
        // You can change the color again on button click if needed
        // For now, let's keep it static
    }

    // Function to convert hex color to Color
    private Color HexToColor(string hex)
    {
        Color color = Color.black;
        ColorUtility.TryParseHtmlString("#" + hex, out color);
        return color;
    }
}
