using UnityEngine;
using TMPro;
using UnityEngine.UI;

public class LapTimer : MonoBehaviour
{
    [Header("References")]
    public Rigidbody carRigidbody;

    [Header("UI")]
    public TMP_Text lapTimeText;
    public TMP_Text sector1Text;
    public TMP_Text sector2Text;
    public TMP_Text sector3Text;
    public TMP_Text lapListText;

    // Timing
    private bool lapRunning = false;
    private float lapStartTime;
    private float currentSectorStartTime;
    private int currentSector = 0;   // 0 = not started, 1,2,3
    private int lapNumber = 0;
    // Best sector times (for the whole session)
    private float bestSector1Time = Mathf.Infinity;
    private float bestSector2Time = Mathf.Infinity;
    private float bestSector3Time = Mathf.Infinity;

    [Header("Sector Backgrounds")]
    public Image sector1Background;
    public Image sector2Background;
    public Image sector3Background;

    [Header("Colors")]
    public Color fastestColor = Color.green;
    public Color slowerColor = Color.yellow;
    public Color defaultColor = new Color(0, 0, 0, 0f);

    private void ResetTimersUI()
    {
        if (lapTimeText != null) lapTimeText.text = "00:00.000";
        if (sector1Text != null) sector1Text.text = "00.000";
        if (sector2Text != null) sector2Text.text = "00.000";
        if (sector3Text != null) sector3Text.text = "00.000";
    }

    private void ResetSectorBackgrounds()
    {
        if (sector1Background != null) sector1Background.color = defaultColor;
        if (sector1Background != null) sector2Background.color = defaultColor;
        if (sector1Background != null) sector3Background.color = defaultColor;
    }

    private void Start()
    {
        if (carRigidbody == null)
            carRigidbody = GetComponent<Rigidbody>();

        ResetTimersUI();
        HandleFinishLine();
    }

    private void Update()
    {
        // Live lap timer display while lap is running
        if (lapRunning && lapTimeText != null)
        {
            float lapTime = Time.time - lapStartTime;
            lapTimeText.text = FormatTime(lapTime);
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Finish"))
        {
            HandleFinishLine();
        }
        else if (other.CompareTag("Sector1"))
        {
            HandleSector(1);
        }
        else if (other.CompareTag("Sector2"))
        {
            HandleSector(2);
        }
    }

    private void HandleFinishLine()
    {
        if (!lapRunning)
        {
            // FIRST time crossing: start lap
            lapRunning = true;
            lapStartTime = Time.time;
            currentSectorStartTime = Time.time;
            currentSector = 1;

            // optional: reset on first lap
            ResetTimersUI();
            return;
        }

        if (currentSector == 3)
        {
            float sector3Time = Time.time - currentSectorStartTime;
            if (sector3Text != null)
                sector3Text.text = FormatTime(sector3Time);
            if (sector3Background != null)
            {
                if (sector3Time < bestSector3Time)
                {
                    bestSector3Time = sector3Time;
                    sector3Background.color = fastestColor; // green
                }
                else
                {
                    sector3Background.color = slowerColor;  // yellow
                }
            }


            float lapTime = Time.time - lapStartTime;
            if (lapTimeText != null)
                lapTimeText.text = FormatTime(lapTime);

            lapNumber++;
            if (lapListText != null)
            {
                string line = $"LAP {lapNumber}: {FormatTime(lapTime)}";

                if (string.IsNullOrEmpty(lapListText.text))
                    lapListText.text = line;
                else
                    lapListText.text += "\n" + line;
            }

            // Immediately start new lap
            lapStartTime = Time.time;
            currentSectorStartTime = Time.time;
            currentSector = 1;
        }
    }

    private void HandleSector(int sectorIndex)
    {
        if (!lapRunning)
            return;

        float now = Time.time;
        float sectorTime = now - currentSectorStartTime;

        if (sectorIndex == 1 && currentSector == 1)
        {
            if (sector1Text != null)
                sector1Text.text = FormatTime(sectorTime);

            // ✅ ADD THIS INSIDE HERE (Sector 1 color)
            if (sector1Background != null)
            {
                if (sectorTime < bestSector1Time)
                {
                    bestSector1Time = sectorTime;
                    sector1Background.color = fastestColor;   // green
                }
                else
                {
                    sector1Background.color = slowerColor;    // yellow
                }
            }

            currentSectorStartTime = now;
            currentSector = 2;
        }
        else if (sectorIndex == 2 && currentSector == 2)
        {
            if (sector2Text != null)
                sector2Text.text = FormatTime(sectorTime);

            // ✅ ADD THIS INSIDE HERE (Sector 2 color)
            if (sector2Background != null)
            {
                if (sectorTime < bestSector2Time)
                {
                    bestSector2Time = sectorTime;
                    sector2Background.color = fastestColor;   // green
                }
                else
                {
                    sector2Background.color = slowerColor;    // yellow
                }
            }

            currentSectorStartTime = now;
            currentSector = 3;
        }
        // If triggers are hit out of order, we just ignore them.
    }



    private string FormatTime(float t)
    {
        int minutes = Mathf.FloorToInt(t / 60f);
        float seconds = t % 60f;
        // "1:23.195"
        return $"{minutes}:{seconds:00.000}";
    }
}
