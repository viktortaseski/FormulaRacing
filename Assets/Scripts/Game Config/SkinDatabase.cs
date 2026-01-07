using System;
using UnityEngine;

[CreateAssetMenu(menuName = "FormulaRacing/Skin Database", fileName = "SkinDatabase")]
public class SkinDatabase : ScriptableObject
{
    public SkinEntry[] skins;

    public SkinEntry GetById(string id)
    {
        if (skins == null)
        {
            return null;
        }

        foreach (var skin in skins)
        {
            if (skin != null && skin.id == id)
            {
                return skin;
            }
        }

        return null;
    }

    public SkinEntry GetByIndex(int index)
    {
        if (skins == null || index < 0 || index >= skins.Length)
        {
            return null;
        }

        return skins[index];
    }
}

[Serializable]
public class SkinEntry
{
    public string id;
    public string displayName;
    public GameObject prefab;
    public Sprite preview;
}
