using UnityEngine;

[DisallowMultipleComponent]
public class AudioManager : MonoBehaviour
{
    private static AudioManager instance;

    [SerializeField] private AudioSource themeSong;

    private void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
        DontDestroyOnLoad(gameObject);
    }

    private void Start()
    {
        if (themeSong != null && !themeSong.isPlaying)
            themeSong.Play();
    }
}
