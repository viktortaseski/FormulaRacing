using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class StartSemaphore : MonoBehaviour
{
    [Header("Lights")]
    [SerializeField] private Image[] lights;
    [SerializeField] private float lightIntervalSeconds = 0.6f;
    [SerializeField] private float allLitHoldSeconds = 0.4f;

    [Header("Audio")]
    [SerializeField] private AudioSource audioSource;
    [SerializeField] private AudioClip lightOnClip;

    [Header("Visibility")]
    [SerializeField] private GameObject rootToHide;
    [SerializeField] private bool autoStart = true;

    /// <summary>
    /// True while the start countdown is running. Input controllers check this
    /// to keep the car still until the lights go out — works across objects and
    /// for runtime-spawned cars, with no Inspector references needed.
    /// </summary>
    public static bool ControlsLocked { get; private set; }

    private Coroutine sequenceRoutine;

    private void Awake()
    {
        // Reset in case a previous scene left it locked (static persists).
        ControlsLocked = false;

        if (rootToHide == null)
            rootToHide = gameObject;
    }

    private void Start()
    {
        if (autoStart)
            BeginSequence();
    }

    public void BeginSequence()
    {
        if (sequenceRoutine != null)
            StopCoroutine(sequenceRoutine);

        sequenceRoutine = StartCoroutine(RunSequence());
    }

    public void ShowAndStart()
    {
        if (rootToHide != null)
            rootToHide.SetActive(true);

        BeginSequence();
    }

    private IEnumerator RunSequence()
    {
        ControlsLocked = true;
        SetAllLights(false);

        for (int i = 0; i < lights.Length; i++)
        {
            if (lights[i] != null)
                lights[i].enabled = true;

            if (audioSource != null && lightOnClip != null)
                audioSource.PlayOneShot(lightOnClip);

            yield return new WaitForSeconds(lightIntervalSeconds);
        }

        if (allLitHoldSeconds > 0f)
            yield return new WaitForSeconds(allLitHoldSeconds);

        SetAllLights(false);
        ControlsLocked = false;

        if (rootToHide != null)
            rootToHide.SetActive(false);

        sequenceRoutine = null;
    }

    private void SetAllLights(bool enabled)
    {
        if (lights == null)
            return;

        for (int i = 0; i < lights.Length; i++)
        {
            if (lights[i] != null)
                lights[i].enabled = enabled;
        }
    }

}
