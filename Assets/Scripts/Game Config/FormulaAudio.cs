using UnityEngine;

public class FormulaAudio : MonoBehaviour
{
    [Header("References")]
    public SimpleCarController carController;
    public Rigidbody carRigidbody;
    public AudioSource engineIdleSource;
    public AudioSource engineAccelSource;
    public AudioSource gearShiftSource;

    [Header("Sound Tuning")]
    public float minPitch = 0.85f;      // idle
    public float maxPitch = 2.2f;       // 15k RPM
    public float accelVolumeBoost = 1.2f;
    [Tooltip("Speed (m/s) that maps to max pitch.")]
    public float maxSpeed = 80f;

    void Start()
    {
        if (carRigidbody == null && carController != null)
            carRigidbody = carController.Rigidbody;

        if (carRigidbody == null)
            carRigidbody = GetComponent<Rigidbody>();

        if (engineIdleSource != null)
            engineIdleSource.Play();

        if (engineAccelSource != null)
            engineAccelSource.Play();
    }

    void Update()
    {
        if (carRigidbody == null) return;

        var speed = carRigidbody.linearVelocity.magnitude;
        UpdateEngineSounds(speed);
    }

    private void UpdateEngineSounds(float speed)
    {
        float t = Mathf.InverseLerp(0f, maxSpeed, speed);

        // Crossfade idle <-> acceleration
        if (engineIdleSource != null)
            engineIdleSource.volume = 1f - t;

        if (engineAccelSource != null)
            engineAccelSource.volume = t * accelVolumeBoost;

        // Pitch scaling
        float pitch = Mathf.Lerp(minPitch, maxPitch, t);
        if (engineIdleSource != null)
            engineIdleSource.pitch = pitch * 0.9f;

        if (engineAccelSource != null)
            engineAccelSource.pitch = pitch;
    }
}
