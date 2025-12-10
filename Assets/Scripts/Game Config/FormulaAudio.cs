using UnityEngine;

public class FormulaAudio : MonoBehaviour
{
    [Header("References")]
    public FormulaPhysics physics;   // << important
    public AudioSource engineIdleSource;
    public AudioSource engineAccelSource;
    public AudioSource gearShiftSource;

    [Header("Sound Tuning")]
    public float minPitch = 0.85f;      // idle
    public float maxPitch = 2.2f;       // 15k RPM
    public float accelVolumeBoost = 1.2f;

    private int lastGear = 0;

    void Start()
    {
        engineIdleSource.Play();
        engineAccelSource.Play();

        if (physics != null)
            lastGear = physics.currentGear;
    }

    void Update()
    {
        if (physics == null) return;

        UpdateEngineSounds(physics.engineRPM);
        DetectGearShift(physics.currentGear);
    }

    private void UpdateEngineSounds(float rpm)
    {
        float t = Mathf.InverseLerp(physics.minEngineRPM, physics.maxEngineRPM, rpm);

        // Crossfade idle <-> acceleration
        engineIdleSource.volume = 1f - t;
        engineAccelSource.volume = t * accelVolumeBoost;

        // Pitch scaling
        float pitch = Mathf.Lerp(minPitch, maxPitch, t);
        engineIdleSource.pitch = pitch * 0.9f;
        engineAccelSource.pitch = pitch;
    }

    private void DetectGearShift(int currentGear)
    {
        if (currentGear != lastGear)
        {
            // Play gearshift SOUND, not loop
            if (!gearShiftSource.isPlaying)
                gearShiftSource.Play();

            lastGear = currentGear;
        }
    }
}
