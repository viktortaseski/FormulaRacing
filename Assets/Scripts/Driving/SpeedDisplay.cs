using UnityEngine;
using UnityEngine.InputSystem;
using UnityEngine.EventSystems;
using TMPro;

public class SpeedDisplay : MonoBehaviour
{
    [Header("Target")]
    public Rigidbody targetRigidbody;   // car rigidbody

    [Header("UI")]
    public TMP_Text speedText;              // UI Text on Canvas

    [Header("Settings")]
    public bool showInKPH = true;       // toggle km/h vs m/s

    private void Update()
    {
        if (targetRigidbody == null || speedText == null)
            return;

        // Speed in m/s
        float speedMS = targetRigidbody.linearVelocity.magnitude;

        float speed = showInKPH ? speedMS * 3.6f : speedMS;
        string unit = showInKPH ? "km/h" : "m/s";

        speedText.text = $"{speed:0} {unit}";
    }
}
