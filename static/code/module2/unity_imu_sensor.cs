using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

/// <summary>
/// Simulates IMU sensor with realistic noise and bias
/// </summary>
public class ROSImuSensor : MonoBehaviour
{
    private ROSConnection ros;
    private Rigidbody rb;

    [Header("ROS Configuration")]
    public string topicName = "/robot/imu/data";
    public float publishRate = 100f;  // Hz

    [Header("Noise Parameters")]
    public float accelNoise = 0.01f;    // m/s²
    public float accelBias = 0.05f;
    public float gyroNoise = 0.001f;    // rad/s
    public float gyroBias = 0.0001f;

    private Vector3 accelBiasOffset;
    private Vector3 gyroBiasOffset;
    private Vector3 lastVelocity;
    private float timeElapsed;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImuMsg>(topicName);

        // Initialize biases (constant for this session)
        accelBiasOffset = new Vector3(
            Random.Range(-accelBias, accelBias),
            Random.Range(-accelBias, accelBias),
            Random.Range(-accelBias, accelBias)
        );

        gyroBiasOffset = new Vector3(
            Random.Range(-gyroBias, gyroBias),
            Random.Range(-gyroBias, gyroBias),
            Random.Range(-gyroBias, gyroBias)
        );

        lastVelocity = rb != null ? rb.velocity : Vector3.zero;

        Debug.Log($"IMU sensor initialized: {topicName} at {publishRate} Hz");
    }

    void FixedUpdate()
    {
        timeElapsed += Time.fixedDeltaTime;

        if (timeElapsed >= 1f / publishRate)
        {
            PublishIMU();
            timeElapsed = 0;
        }
    }

    void PublishIMU()
    {
        // Compute linear acceleration (includes gravity)
        Vector3 accel = rb != null ?
            (rb.velocity - lastVelocity) / Time.fixedDeltaTime :
            Vector3.zero;

        // Add gravity in world frame
        accel += Physics.gravity;

        // Convert to sensor frame (assuming sensor aligned with body)
        accel = transform.InverseTransformDirection(accel);

        // Add noise and bias
        accel += accelBiasOffset + new Vector3(
            GaussianRandom(0, accelNoise),
            GaussianRandom(0, accelNoise),
            GaussianRandom(0, accelNoise)
        );

        lastVelocity = rb != null ? rb.velocity : Vector3.zero;

        // Angular velocity from Unity (convert to sensor frame)
        Vector3 angularVel = rb != null ?
            transform.InverseTransformDirection(rb.angularVelocity) :
            Vector3.zero;

        angularVel += gyroBiasOffset + new Vector3(
            GaussianRandom(0, gyroNoise),
            GaussianRandom(0, gyroNoise),
            GaussianRandom(0, gyroNoise)
        );

        // Orientation from Unity transform
        Quaternion orientation = transform.rotation;

        // Create ROS message
        ImuMsg msg = new ImuMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = ROSClock.Now(),
                frame_id = "imu_link"
            },
            orientation = new RosMessageTypes.Geometry.QuaternionMsg
            {
                x = orientation.x,
                y = orientation.y,
                z = orientation.z,
                w = orientation.w
            },
            angular_velocity = new RosMessageTypes.Geometry.Vector3Msg
            {
                x = angularVel.x,
                y = angularVel.y,
                z = angularVel.z
            },
            linear_acceleration = new RosMessageTypes.Geometry.Vector3Msg
            {
                x = accel.x,
                y = accel.y,
                z = accel.z
            }
        };

        // Covariance matrices (diagonal, representing noise variance)
        double accelVar = accelNoise * accelNoise;
        double gyroVar = gyroNoise * gyroNoise;
        double orientVar = 0.01;  // Orientation uncertainty

        msg.orientation_covariance = new double[]
        {
            orientVar, 0, 0,
            0, orientVar, 0,
            0, 0, orientVar
        };

        msg.angular_velocity_covariance = new double[]
        {
            gyroVar, 0, 0,
            0, gyroVar, 0,
            0, 0, gyroVar
        };

        msg.linear_acceleration_covariance = new double[]
        {
            accelVar, 0, 0,
            0, accelVar, 0,
            0, 0, accelVar
        };

        ros.Publish(topicName, msg);
    }

    float GaussianRandom(float mean, float stdDev)
    {
        // Box-Muller transform
        float u1 = Random.value;
        float u2 = Random.value;
        float randStdNormal = Mathf.Sqrt(-2.0f * Mathf.Log(u1)) *
                              Mathf.Sin(2.0f * Mathf.PI * u2);
        return mean + stdDev * randStdNormal;
    }
}
