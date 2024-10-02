using UnityEngine;
using Microsoft.MixedReality.Toolkit.Input;
using System.Net.Sockets;
using System.Text;

public class Click : MonoBehaviour, IMixedRealityPointerHandler
{
    public float highlightRadius = 0.05f; // Radius around the hit point to highlight
    public Material highlightMaterial; // Red material to apply for highlighting

    private TcpClient coordClient;
    private NetworkStream coordStream;
    private string xavierIP = "192.168.1.134"; // Xavier's IP address
    private int coordPort = 9998; // Port for sending coordinates

    void Start()
    {
        // Initialize the TCP connection on start
        ConnectToServer();
    }

    async void ConnectToServer()
    {
        coordClient = new TcpClient();
        try
        {
            await coordClient.ConnectAsync(xavierIP, coordPort);
            coordStream = coordClient.GetStream();
            Debug.Log("Connected to Xavier server.");
        }
        catch (SocketException e)
        {
            Debug.LogError($"SocketException: {e.ToString()}");
        }
    }

    public void OnPointerClicked(MixedRealityPointerEventData eventData)
    {
        var result = eventData.Pointer.Result;
        var hitPoint = result.Details.Point; // World space hit point
        HighlightArea(hitPoint);

        Vector3 localHitPoint = transform.InverseTransformPoint(hitPoint);
        float newX = (localHitPoint.x + 5) / 10;
        float newZ = (5 - localHitPoint.z) / 10;

        int frameX = Mathf.RoundToInt(newX * 640);
        int frameZ = Mathf.RoundToInt(newZ * 480);

        frameX = Mathf.Clamp(frameX, 0, 640 - 1);
        frameZ = Mathf.Clamp(frameZ, 0, 480 - 1);

        SendCoordinates(frameX, frameZ);
    }

    async void SendCoordinates(int x, int z)
    {
        if (coordClient != null && coordClient.Connected)
        {
            string message = $"({x},{z})";
            byte[] data = Encoding.ASCII.GetBytes(message);
            await coordStream.WriteAsync(data, 0, data.Length);
        }
        else
        {
            Debug.LogWarning("Not connected to the server.");
            ConnectToServer(); // Attempt to reconnect
        }
    }

    private void HighlightArea(Vector3 centerPoint)
    {
        GameObject highlightObject = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        Destroy(highlightObject.GetComponent<Collider>()); // Remove collider
        highlightObject.transform.position = centerPoint;
        highlightObject.transform.localScale = new Vector3(highlightRadius, highlightRadius, 0.001f);
        highlightObject.GetComponent<Renderer>().material = highlightMaterial;

        Destroy(highlightObject, 2.0f); // Schedule destruction in 2 seconds
    }

    void OnDestroy()
    {
        if (coordStream != null)
        {
            coordStream.Close();
        }
        if (coordClient != null)
        {
            coordClient.Close();
        }
    }

    public void OnPointerDown(MixedRealityPointerEventData eventData) { }
    public void OnPointerDragged(MixedRealityPointerEventData eventData) { }
    public void OnPointerUp(MixedRealityPointerEventData eventData) { }
}
