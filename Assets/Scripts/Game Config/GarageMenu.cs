using UnityEngine;
using UnityEngine.UI;

public class GarageMenu : MonoBehaviour
{
    [SerializeField] private GameObject mainMenuPanel;
    [SerializeField] private GameObject garagePanel;


    public void Open()
    {
        if (mainMenuPanel != null)
        {
            mainMenuPanel.SetActive(false);
        }

        if (garagePanel != null)
        {
            garagePanel.SetActive(true);
        }

    }

    public void Close()
    {
        if (garagePanel != null)
        {
            garagePanel.SetActive(false);
        }

        if (mainMenuPanel != null)
        {
            mainMenuPanel.SetActive(true);
        }
    }

}
