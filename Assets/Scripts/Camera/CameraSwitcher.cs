using UnityEngine;

public class CameraSwitcher : MonoBehaviour
{
    public Camera camera1;
    public Camera camera2;
    public Camera camera3;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        ActivateCamera(1);
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha1))
            ActivateCamera(1);
        if (Input.GetKeyDown(KeyCode.Alpha2))
            ActivateCamera(2);
        if (Input.GetKeyDown(KeyCode.Alpha3))
            ActivateCamera(3);
    }

    void ActivateCamera(int camNumber)
    {
        camera1.enabled = (camNumber == 1);
        camera2.enabled = (camNumber == 2);
        camera3.enabled = (camNumber == 3);
    }
}
