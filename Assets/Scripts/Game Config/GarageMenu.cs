using UnityEngine;
using UnityEngine.UI;

public class GarageMenu : MonoBehaviour
{
    [SerializeField] private GameObject mainMenuPanel;
    [SerializeField] private GameObject garagePanel;


    public void Open() => SetVisible(true);
    public void Close() => SetVisible(false);

    private void SetVisible(bool open)
    {
        if (mainMenuPanel != null) mainMenuPanel.SetActive(!open);
        if (garagePanel != null) garagePanel.SetActive(open);
    }

}
