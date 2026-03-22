using UnityEngine;
using UnityEngine.SceneManagement;

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
    [Header("Scene Control")]
    [SerializeField] private string settingsSceneName = "Settings";

    private void OnEnable()
    {
        SceneManager.activeSceneChanged += OnActiveSceneChanged;
    }

    private void OnDisable()
    {
        SceneManager.activeSceneChanged -= OnActiveSceneChanged;
    }

    void Start()
    {
        if (carRigidbody == null && carController != null)
            carRigidbody = carController.Rigidbody;

        if (carRigidbody == null)
            carRigidbody = GetComponent<Rigidbody>();

        SyncAudioState();
    }

    void Update()
    {
        if (IsSettingsSceneActive())
            return;

        if (carRigidbody == null) return;

        var speed = carRigidbody.linearVelocity.magnitude;
        UpdateEngineSounds(speed);
    }

    private void OnActiveSceneChanged(Scene current, Scene next)
    {
        SyncAudioState();
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

    private void SyncAudioState()
    {
        if (IsSettingsSceneActive())
        {
            PauseSource(engineIdleSource);
            PauseSource(engineAccelSource);
            PauseSource(gearShiftSource);
            return;
        }

        ResumeOrPlaySource(engineIdleSource);
        ResumeOrPlaySource(engineAccelSource);
    }

    private bool IsSettingsSceneActive()
    {
        if (string.IsNullOrEmpty(settingsSceneName))
            return false;

        return SceneManager.GetActiveScene().name == settingsSceneName;
    }

    public void StopEngine()
    {
        PauseSource(engineIdleSource);
        PauseSource(engineAccelSource);
        PauseSource(gearShiftSource);
    }

    private void PauseSource(AudioSource source)
    {
        if (source != null && source.isPlaying)
            source.Pause();
    }

    private void ResumeOrPlaySource(AudioSource source)
    {
        if (source == null || source.isPlaying)
            return;

        source.UnPause();

        if (!source.isPlaying)
            source.Play();
    }
}
