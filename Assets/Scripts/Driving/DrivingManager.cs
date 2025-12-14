using UnityEngine;

public class DrivingManager : MonoBehaviour
{
    [Header("References")]
    public FormulaPhysics car;

    [Header("Grass Settings")]
    public Collider grassPlaneCollider;
    [Range(0.1f, 1f)] public float grassGripMultiplier = 0.5f;
    [Range(0.1f, 1f)] public float grassAccelerationMultiplier = 0.6f;

    private bool _isOnGrass;
    private float _baseEngineTorqueMultiplier;
    private float _baseForwardStiffness;
    private float _baseSidewaysStiffness;

    private void Awake()
    {
        if (car == null)
            car = GetComponent<FormulaPhysics>();

        CacheBaseValues();
    }

    private void OnDisable()
    {
        if (_isOnGrass && car != null)
            RestoreBaseHandling();
    }

    private void FixedUpdate()
    {
        if (car == null)
            return;

        bool allWheelsOnGrass = AllWheelsOnGrass();

        if (allWheelsOnGrass && !_isOnGrass)
        {
            CacheBaseValues();
            ApplyGrassHandling();
            _isOnGrass = true;
        }
        else if (!allWheelsOnGrass && _isOnGrass)
        {
            RestoreBaseHandling();
            _isOnGrass = false;
            CacheBaseValues(); // keep base values in sync after restoring
        }
        else if (!allWheelsOnGrass)
        {
            CacheBaseValues(); // track any runtime changes (e.g., upgrades/DRS)
        }
    }

    private void CacheBaseValues()
    {
        if (car == null)
            return;

        _baseEngineTorqueMultiplier = car.engineTorqueMultiplier;
        _baseForwardStiffness = car.forwardStiffness;
        _baseSidewaysStiffness = car.sidewaysStiffness;
    }

    private void ApplyGrassHandling()
    {
        car.engineTorqueMultiplier = _baseEngineTorqueMultiplier * grassAccelerationMultiplier;
        car.forwardStiffness = _baseForwardStiffness * grassGripMultiplier;
        car.sidewaysStiffness = _baseSidewaysStiffness * grassGripMultiplier;
    }

    private void RestoreBaseHandling()
    {
        car.engineTorqueMultiplier = _baseEngineTorqueMultiplier;
        car.forwardStiffness = _baseForwardStiffness;
        car.sidewaysStiffness = _baseSidewaysStiffness;
    }

    private bool AllWheelsOnGrass()
    {
        return WheelOnGrass(car.frontLeft) &&
               WheelOnGrass(car.frontRight) &&
               WheelOnGrass(car.rearLeft) &&
               WheelOnGrass(car.rearRight);
    }

    private bool WheelOnGrass(WheelCollider wheel)
    {
        if (wheel == null)
            return false;

        if (wheel.GetGroundHit(out WheelHit hit))
        {
            Collider hitCollider = hit.collider;
            return grassPlaneCollider != null && hitCollider == grassPlaneCollider;
        }

        return false;
    }
}
