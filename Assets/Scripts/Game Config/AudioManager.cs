using UnityEngine;
using UnityEngine.SceneManagement;

[DisallowMultipleComponent]
public class AudioManager : MonoBehaviour
{
    private const string MusicEnabledKey = "settings.musicEnabled";
    private static AudioManager instance;

    [SerializeField] private AudioSource themeSong;
    [SerializeField] private bool defaultMusicEnabled = true;
    [SerializeField] private bool includeChildAudioSources = true;

    private AudioSource[] musicSources = new AudioSource[0];

    private void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }

        if (transform.parent != null)
            transform.SetParent(null, true);

        instance = this;
        DontDestroyOnLoad(gameObject);

        CacheSources();
        ApplyMusicEnabled(GetMusicEnabled(defaultMusicEnabled));
    }

    private void Start()
    {
        if (themeSong != null && !themeSong.isPlaying && GetMusicEnabled(defaultMusicEnabled))
            themeSong.Play();
    }

    private void OnEnable()
    {
        SceneManager.sceneLoaded += OnSceneLoaded;
    }

    private void OnDisable()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }

    private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        if (instance != this)
            return;

        CacheSources();
        ApplyMusicEnabled(GetMusicEnabled(defaultMusicEnabled));
    }

    public static void SetMusicEnabled(bool enabled)
    {
        PlayerPrefs.SetInt(MusicEnabledKey, enabled ? 1 : 0);
        PlayerPrefs.Save();

        if (instance != null)
            instance.ApplyMusicEnabled(enabled);
    }

    public void SetMusicEnabledFromUI(bool enabled)
    {
        SetMusicEnabled(enabled);
    }

    public static bool GetMusicEnabled(bool defaultValue)
    {
        return PlayerPrefs.GetInt(MusicEnabledKey, defaultValue ? 1 : 0) == 1;
    }

    private void CacheSources()
    {
        if (includeChildAudioSources)
            musicSources = GetComponentsInChildren<AudioSource>(true);
        else if (themeSong != null)
            musicSources = new[] { themeSong };
        else
            musicSources = GetComponents<AudioSource>();
    }

    private void ApplyMusicEnabled(bool enabled)
    {
        if (musicSources == null || musicSources.Length == 0)
            return;

        for (int i = 0; i < musicSources.Length; i++)
        {
            var source = musicSources[i];
            if (source == null)
                continue;

            source.mute = !enabled;

            if (enabled)
            {
                if (!source.isPlaying)
                    source.Play();
            }
            else
            {
                if (source.isPlaying)
                    source.Pause();
            }
        }
    }
}
