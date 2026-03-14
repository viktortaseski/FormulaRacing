using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public class RestartManager : MonoBehaviour
{
    private static string lastSceneName;
    private static bool settingsLoadedAdditively;
    private static EventSystem[] cachedEventSystems;
    private static bool[] cachedEventSystemStates;

    [SerializeField] private bool enableKeyboardInput = true;
    [SerializeField] private KeyCode restartKey = KeyCode.R;
    [SerializeField] private string mainMenuSceneName = "MainMenu";
    [SerializeField] private string settingsSceneName = "Settings";

    private void Update()
    {
        HandleKeyboardInput();
    }

    public void RestartScene()
    {
        Time.timeScale = 1f;
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }

    public void LoadMainMenu()
    {
        Time.timeScale = 1f;
        lastSceneName = string.Empty;
        settingsLoadedAdditively = false;
        if (!string.IsNullOrEmpty(mainMenuSceneName))
        {
            SceneManager.LoadScene(mainMenuSceneName);
        }
    }

    public void LoadSettings()
    {
        CacheLastScene();
        if (!string.IsNullOrEmpty(settingsSceneName))
        {
            if (IsActiveSceneMainMenu())
            {
                settingsLoadedAdditively = false;
                SceneManager.LoadScene(settingsSceneName);
                return;
            }

            var settingsScene = SceneManager.GetSceneByName(settingsSceneName);
            if (settingsScene.isLoaded)
            {
                SceneManager.SetActiveScene(settingsScene);
                return;
            }

            settingsLoadedAdditively = true;
            CacheEventSystems();
            var load = SceneManager.LoadSceneAsync(settingsSceneName, LoadSceneMode.Additive);
            if (load != null)
            {
                load.completed += _ =>
                {
                    var loadedScene = SceneManager.GetSceneByName(settingsSceneName);
                    if (loadedScene.IsValid())
                    {
                        SceneManager.SetActiveScene(loadedScene);
                        EnableOnlyEventSystemInScene(loadedScene);
                    }
                };
            }
        }
    }

    public void BackToRacing()
    {
        if (settingsLoadedAdditively)
        {
            var settingsScene = SceneManager.GetSceneByName(settingsSceneName);
            if (settingsScene.isLoaded)
            {
                SceneManager.UnloadSceneAsync(settingsScene);
            }

            RestoreEventSystems();
            settingsLoadedAdditively = false;

            if (!string.IsNullOrEmpty(lastSceneName))
            {
                var lastScene = SceneManager.GetSceneByName(lastSceneName);
                if (lastScene.IsValid())
                    SceneManager.SetActiveScene(lastScene);
            }

            ApplyGameplayPauseState();
            return;
        }

        if (!string.IsNullOrEmpty(lastSceneName))
        {
            SceneManager.LoadScene(lastSceneName);
            return;
        }

        LoadMainMenu();
    }

    private void ApplyGameplayPauseState()
    {
        Time.timeScale = PauseMenuController.HasVisibleMenuPanel() ? 0f : 1f;
    }

    private void HandleKeyboardInput()
    {
        if (!enableKeyboardInput)
            return;

        if (Input.GetKeyDown(restartKey))
            RestartScene();
    }

    private void CacheLastScene()
    {
        var active = SceneManager.GetActiveScene();
        if (!active.IsValid())
            return;

        if (!string.IsNullOrEmpty(settingsSceneName) && active.name == settingsSceneName)
            return;

        lastSceneName = active.name;
    }

    private bool IsActiveSceneMainMenu()
    {
        var active = SceneManager.GetActiveScene();
        return active.IsValid() && active.name == mainMenuSceneName;
    }

    private void CacheEventSystems()
    {
        cachedEventSystems = Resources.FindObjectsOfTypeAll<EventSystem>();
        cachedEventSystemStates = new bool[cachedEventSystems.Length];
        for (int i = 0; i < cachedEventSystems.Length; i++)
            cachedEventSystemStates[i] = cachedEventSystems[i] != null && cachedEventSystems[i].enabled;
    }

    private void RestoreEventSystems()
    {
        if (cachedEventSystems == null || cachedEventSystemStates == null)
            return;

        for (int i = 0; i < cachedEventSystems.Length; i++)
        {
            if (cachedEventSystems[i] != null)
                cachedEventSystems[i].enabled = cachedEventSystemStates[i];
        }
    }

    private void EnableOnlyEventSystemInScene(Scene scene)
    {
        var all = Resources.FindObjectsOfTypeAll<EventSystem>();
        for (int i = 0; i < all.Length; i++)
        {
            if (all[i] == null)
                continue;

            all[i].enabled = all[i].gameObject.scene == scene;
        }
    }
}
