using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class CarController : MonoBehaviour
{
    [Header("Wheel Colliders")]
    public WheelCollider frontLeftWheelCollider;
    public WheelCollider frontRightWheelCollider;
    public WheelCollider rearLeftWheelCollider;
    public WheelCollider rearRightWheelCollider;

    [Header("Wheel Meshes (visual)")]
    public Transform frontLeftWheelMesh;
    public Transform frontRightWheelMesh;
    public Transform rearLeftWheelMesh;
    public Transform rearRightWheelMesh;

    [Header("Car Settings")]
    public float motorTorque = 1500f;    // how strong the acceleration is
    public float maxSteerAngle = 25f;    // steering angle in degrees
    public float brakeTorque = 3000f;    // handbrake strength

    private float horizontalInput;       // A/D or Left/Right arrows
    private float verticalInput;         // W/S or Up/Down arrows
    private bool isBraking;

    private void Update()
    {
        // ---- INPUT (default Input System) ----
        horizontalInput = Input.GetAxis("Horizontal"); // -1 .. 1
        verticalInput   = Input.GetAxis("Vertical");   // -1 .. 1
        isBraking       = Input.GetKey(KeyCode.Space); // Space = handbrake

        // Update wheel meshes visually
        UpdateWheelMeshes();
    }

    private void FixedUpdate()
    {
        HandleMotor();
        HandleSteering();
        HandleBraking();
    }

    private void HandleMotor()
    {
        float torque = verticalInput * motorTorque;

        // Rear-wheel drive
        rearLeftWheelCollider.motorTorque  = torque;
        rearRightWheelCollider.motorTorque = torque;
    }

    private void HandleSteering()
    {
        float steer = horizontalInput * maxSteerAngle;

        frontLeftWheelCollider.steerAngle  = steer;
        frontRightWheelCollider.steerAngle = steer;
    }

    private void HandleBraking()
    {
        float currentBrakeTorque = isBraking ? brakeTorque : 0f;

        frontLeftWheelCollider.brakeTorque  = currentBrakeTorque;
        frontRightWheelCollider.brakeTorque = currentBrakeTorque;
        rearLeftWheelCollider.brakeTorque   = currentBrakeTorque;
        rearRightWheelCollider.brakeTorque  = currentBrakeTorque;
    }

    private void UpdateWheelMeshes()
    {
        UpdateSingleWheel(frontLeftWheelCollider,  frontLeftWheelMesh);
        UpdateSingleWheel(frontRightWheelCollider, frontRightWheelMesh);
        UpdateSingleWheel(rearLeftWheelCollider,   rearLeftWheelMesh);
        UpdateSingleWheel(rearRightWheelCollider,  rearRightWheelMesh);
    }

    private void UpdateSingleWheel(WheelCollider col, Transform mesh)
    {
        if (col == null || mesh == null) return;

        col.GetWorldPose(out Vector3 pos, out Quaternion rot);
        mesh.position = pos;
        mesh.rotation = rot;
    }
}
