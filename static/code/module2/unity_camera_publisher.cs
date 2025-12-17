using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Publishes camera images to ROS with realistic noise
/// </summary>
public class ROSCameraPublisher : MonoBehaviour
{
    private Camera cam;
    private ROSConnection ros;

    [Header("ROS Configuration")]
    public string topicName = "/robot/camera/image_raw";

    [Header("Camera Parameters")]
    public int width = 1920;
    public int height = 1080;
    public float fov = 80f;  // degrees
    public float publishRate = 30f;  // Hz

    [Header("Noise Parameters")]
    public float noiseStdDev = 0.007f;
    public bool enableNoise = true;

    private float timeElapsed;

    void Start()
    {
        // Get or create camera
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            cam = gameObject.AddComponent<Camera>();
        }

        // Configure camera
        cam.fieldOfView = fov;
        cam.nearClipPlane = 0.1f;
        cam.farClipPlane = 100f;

        // Create render texture
        RenderTexture rt = new RenderTexture(width, height, 24);
        cam.targetTexture = rt;

        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        Debug.Log($"Camera publisher initialized: {topicName} at {publishRate} Hz");
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= 1f / publishRate)
        {
            PublishImage();
            timeElapsed = 0;
        }
    }

    void PublishImage()
    {
        // Render camera view
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = cam.targetTexture;
        cam.Render();

        // Read pixels
        Texture2D image = new Texture2D(width, height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);

        // Add Gaussian noise if enabled
        if (enableNoise && noiseStdDev > 0)
        {
            AddGaussianNoise(image, noiseStdDev);
        }

        image.Apply();

        // Convert to ROS message
        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "camera_optical_frame"
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            step = (uint)(width * 3),
            data = image.GetRawTextureData()
        };

        // Publish
        ros.Publish(topicName, msg);

        RenderTexture.active = currentRT;
        Destroy(image);
    }

    void AddGaussianNoise(Texture2D texture, float stdDev)
    {
        Color[] pixels = texture.GetPixels();
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i].r = Mathf.Clamp01(pixels[i].r + GaussianRandom(0, stdDev));
            pixels[i].g = Mathf.Clamp01(pixels[i].g + GaussianRandom(0, stdDev));
            pixels[i].b = Mathf.Clamp01(pixels[i].b + GaussianRandom(0, stdDev));
        }
        texture.SetPixels(pixels);
    }

    float GaussianRandom(float mean, float stdDev)
    {
        float u1 = Random.value;
        float u2 = Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                              Mathf.Sin(2.0f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }
}
