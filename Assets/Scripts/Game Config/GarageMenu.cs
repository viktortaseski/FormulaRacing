using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class GarageMenu : MonoBehaviour
{
    [SerializeField] private SkinDatabase skinDatabase;
    [SerializeField] private Transform buttonContainer;
    [SerializeField] private Button buttonPrefab;
    [SerializeField] private Image selectedPreviewImage;
    [SerializeField] private TMP_Text selectedNameText;
    [SerializeField] private Button freePracticeButton;
    [SerializeField] private string defaultSkinId = "03_pse_red_white";
    [SerializeField] private GameObject mainMenuPanel;
    [SerializeField] private GameObject garagePanel;

    private bool built;
    private string currentSkinId;

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

        BuildButtonsIfNeeded();
        SelectInitialSkin();
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

    private void BuildButtonsIfNeeded()
    {
        if (built)
        {
            return;
        }

        if (skinDatabase == null || skinDatabase.skins == null)
        {
            Debug.LogError("SkinDatabase is missing on GarageMenu.");
            return;
        }

        if (buttonContainer == null || buttonPrefab == null)
        {
            Debug.LogError("Button container or prefab is missing on GarageMenu.");
            return;
        }

        foreach (var skin in skinDatabase.skins)
        {
            if (skin == null)
            {
                continue;
            }

            var button = Instantiate(buttonPrefab, buttonContainer);
            button.onClick.AddListener(() => SelectSkin(skin));

            var image = button.GetComponent<Image>();
            if (image != null && skin.preview != null)
            {
                image.sprite = skin.preview;
                image.preserveAspect = true;
            }

            var label = button.GetComponentInChildren<TMP_Text>();
            if (label != null)
            {
                label.text = skin.displayName;
            }
        }

        built = true;
    }

    private void SelectInitialSkin()
    {
        var storedId = SkinSelection.GetSelectedId(defaultSkinId);
        var skin = skinDatabase.GetById(storedId) ?? skinDatabase.GetByIndex(0);
        if (skin != null)
        {
            SelectSkin(skin);
        }
    }

    private void SelectSkin(SkinEntry skin)
    {
        if (skin == null)
        {
            return;
        }

        currentSkinId = skin.id;
        SkinSelection.SetSelectedId(currentSkinId);

        if (selectedPreviewImage != null && skin.preview != null)
        {
            selectedPreviewImage.sprite = skin.preview;
            selectedPreviewImage.preserveAspect = true;
        }

        if (selectedNameText != null)
        {
            selectedNameText.text = skin.displayName;
        }

        if (freePracticeButton != null)
        {
            freePracticeButton.interactable = true;
        }
    }
}
