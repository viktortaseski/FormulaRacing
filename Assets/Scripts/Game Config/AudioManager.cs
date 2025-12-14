using UnityEngine;

public class AudioManager : MonoBehaviour
{

    public AudioSource themeSong;

    void Start()
    {
        if (themeSong != null && !themeSong.isPlaying)
        {
            themeSong.Play();
        }
    }

    // Update is called once per frame
    void Update()
    {

    }
}
