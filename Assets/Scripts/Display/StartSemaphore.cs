using System.Collections;
using UnityEngine;
using UnityEngine.UI;

public class StartSemaphore : MonoBehaviour
{
    [Header("Lights")]
    [SerializeField] private Image[] lights;
    [SerializeField] private float lightIntervalSeconds = 0.6f;
    [SerializeField] private float allLitHoldSeconds = 0.4f;

    [Header("Visibility")]
    [SerializeField] private GameObject rootToHide;
    [SerializeField] private bool autoStart = true;

    [Header("Control Lock")]
    [SerializeField] private Behaviour[] disableWhileCounting;

    private Coroutine sequenceRoutine;

    private void Awake()
    {
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
        SetControlsEnabled(false);
        SetAllLights(false);

        for (int i = 0; i < lights.Length; i++)
        {
            if (lights[i] != null)
                lights[i].enabled = true;

            yield return new WaitForSeconds(lightIntervalSeconds);
        }

        if (allLitHoldSeconds > 0f)
            yield return new WaitForSeconds(allLitHoldSeconds);

        SetAllLights(false);
        SetControlsEnabled(true);

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

    private void SetControlsEnabled(bool enabled)
    {
        if (disableWhileCounting == null)
            return;

        for (int i = 0; i < disableWhileCounting.Length; i++)
        {
            if (disableWhileCounting[i] != null)
                disableWhileCounting[i].enabled = enabled;
        }
    }
}
