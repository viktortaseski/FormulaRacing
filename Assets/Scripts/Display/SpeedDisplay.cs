using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.EventSystems;
using TMPro;

public class SpeedDisplay : MonoBehaviour
{
    [Header("Target")]
    [SerializeField] private Rigidbody targetRigidbody;
    [Header("UI")]
    [SerializeField] private TMP_Text speedText;              // UI Text on Canvas
    [Header("Settings")]
    [SerializeField] private bool showInKPH = true;       // toggle km/h vs m/s

    private Vector3 _lastPosition;
    private bool _hasLastPosition = false;
    private float _currentSpeedMS = 0f;

    private void OnEnable()
    {
        if (targetRigidbody != null)
        {
            _lastPosition = targetRigidbody.position;
            _hasLastPosition = true;
        }
    }

    private void FixedUpdate()
    {
        if (targetRigidbody == null)
            return;

        if (!_hasLastPosition)
        {
            _lastPosition = targetRigidbody.position;
            _hasLastPosition = true;
            return;
        }

        calculateSpeed();
    }

    private void Update()
    {
        if (targetRigidbody == null || speedText == null)
            return;

        // Use measured displacement per physics tick to match on-screen movement.
        float speedMS = _currentSpeedMS;

        float speed = showInKPH ? speedMS * 3.6f : speedMS;
        string unit = showInKPH ? "km/h" : "m/s";

        speedText.text = $"{speed:0} {unit}";
    }

    private void calculateSpeed()
    {
        Vector3 delta = targetRigidbody.position - _lastPosition;
        float frameSpeed = delta.magnitude / Mathf.Max(Time.fixedDeltaTime, 0.0001f);
        _currentSpeedMS = frameSpeed;
        _lastPosition = targetRigidbody.position;
    }
}
