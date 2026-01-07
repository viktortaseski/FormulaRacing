using UnityEngine;
using UnityEngine.SceneManagement;

public class MenuManager : MonoBehaviour
{
    [SerializeField] private GarageMenu garageMenu;

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
        if (garageMenu == null)
        {
            Debug.LogWarning("GarageMenu reference is missing on MenuManager.");
            return;
        }

        garageMenu.Open();
    }

    public void QuitGame()
    {
        Application.Quit();
    }
}
