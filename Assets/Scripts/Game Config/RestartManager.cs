using UnityEngine;
using UnityEngine.SceneManagement;

public class RestartManager : MonoBehaviour
{
    [SerializeField] private bool enableKeyboardInput = true;
    [SerializeField] private KeyCode restartKey = KeyCode.R;

    private void Update()
    {
        HandleKeyboardInput();
    }

    public void RestartScene()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    private void HandleKeyboardInput()
    {
        if (!enableKeyboardInput)
            return;

        if (Input.GetKeyDown(restartKey))
            RestartScene();
    }
}
