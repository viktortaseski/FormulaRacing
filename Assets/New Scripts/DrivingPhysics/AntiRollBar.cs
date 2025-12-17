using UnityEngine;

public class AntiRollBar : MonoBehaviour
{
    [SerializeField] private WheelCollider leftWheel;
    [SerializeField] private WheelCollider rightWheel;
    [SerializeField] private float antiRollForce = 12000f;

    private Rigidbody rb;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        ApplyAntiRoll();
    }

    private void ApplyAntiRoll()
    {
        WheelHit hit;
        float leftTravel = 1.0f;
        float rightTravel = 1.0f;

        bool leftGrounded = leftWheel.GetGroundHit(out hit);
        if (leftGrounded)
            leftTravel = (-leftWheel.transform.InverseTransformPoint(hit.point).y - leftWheel.radius) / leftWheel.suspensionDistance;

        bool rightGrounded = rightWheel.GetGroundHit(out hit);
        if (rightGrounded)
            rightTravel = (-rightWheel.transform.InverseTransformPoint(hit.point).y - rightWheel.radius) / rightWheel.suspensionDistance;

        float antiRoll = (leftTravel - rightTravel) * antiRollForce;

        if (leftGrounded)
            rb.AddForceAtPosition(leftWheel.transform.up * -antiRoll, leftWheel.transform.position);

        if (rightGrounded)
            rb.AddForceAtPosition(rightWheel.transform.up * antiRoll, rightWheel.transform.position);
    }
}
