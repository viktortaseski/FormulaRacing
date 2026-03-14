using UnityEngine;

/// <summary>
/// Prevents the car from launching into the air by:
///   1. Clamping upward velocity so the car can't gain too much vertical speed.
///   2. Applying a downward grounding force when the car is airborne.
/// Uses a downward raycast — no other component references required.
/// </summary>
public class AntiFly : MonoBehaviour
{
    [Header("Ground Detection")]
    [Tooltip("How far down to raycast to check if the car is grounded.")]
    [SerializeField] private float groundCheckDistance = 1.2f;
    [Tooltip("Layers considered as ground.")]
    [SerializeField] private LayerMask groundMask = Physics.DefaultRaycastLayers;

    [Header("Upward Velocity Clamp")]
    [Tooltip("Maximum upward velocity the car is allowed to have (m/s). Prevents initial launch.")]
    [SerializeField] private float maxUpwardVelocity = 2f;

    [Header("Grounding Force")]
    [Tooltip("Downward acceleration applied while airborne (m/s²). Stacks with gravity.")]
    [SerializeField] private float groundingForce = 25f;
    [Tooltip("Minimum speed (kph) before grounding force activates. Avoids affecting slow low-speed bumps.")]
    [SerializeField] private float minSpeedKph = 20f;

    private Rigidbody rb;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }

    private void FixedUpdate()
    {
        if (rb == null)
            return;

        ClampUpwardVelocity();

        if (!IsGrounded() && GetSpeedKph() >= minSpeedKph)
            rb.AddForce(Vector3.down * groundingForce, ForceMode.Acceleration);
    }

    private void ClampUpwardVelocity()
    {
        if (rb.linearVelocity.y > maxUpwardVelocity)
        {
            Vector3 v = rb.linearVelocity;
            v.y = maxUpwardVelocity;
            rb.linearVelocity = v;
        }
    }

    private bool IsGrounded()
    {
        return Physics.Raycast(transform.position, Vector3.down, groundCheckDistance, groundMask, QueryTriggerInteraction.Ignore);
    }

    private float GetSpeedKph()
    {
        return rb.linearVelocity.magnitude * 3.6f;
    }
}
