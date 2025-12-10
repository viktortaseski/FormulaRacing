using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuManager : MonoBehaviour
{
    public void StartFreePractice()
    {
        // Load your driving scene (update the name!)
        SceneManager.LoadScene("RaceTrackScene");
    }

    public void OpenMultiplayer()
    {
        // Placeholder until implemented
        Debug.Log("Multiplayer coming soon!");
    }

    public void OpenGarage()
    {
        // Placeholder until implemented
        Debug.Log("Garage coming soon!");
    }

    public void QuitGame()
    {
        Application.Quit();
    }
}
