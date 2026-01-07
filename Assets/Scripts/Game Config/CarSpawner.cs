using UnityEngine;

public class CarSpawner : MonoBehaviour
{
    [SerializeField] private SkinDatabase skinDatabase;
    [SerializeField] private Transform spawnPoint;
    [SerializeField] private string defaultSkinId = "03_pse_red_white";

    private void Start()
    {
        if (skinDatabase == null)
        {
            Debug.LogError("SkinDatabase is missing on CarSpawner.");
            return;
        }

        var selectedId = SkinSelection.GetSelectedId(defaultSkinId);
        var skin = skinDatabase.GetById(selectedId) ?? skinDatabase.GetByIndex(0);
        if (skin == null || skin.prefab == null)
        {
            Debug.LogError("Selected skin prefab is missing.");
            return;
        }

        var point = spawnPoint != null ? spawnPoint : transform;
        Instantiate(skin.prefab, point.position, point.rotation);
    }
}
