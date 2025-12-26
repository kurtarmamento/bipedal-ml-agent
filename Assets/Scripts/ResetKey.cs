using UnityEngine;

public class ResetKey : MonoBehaviour
{
    public RagdollResetter resetter;
    public KeyCode key = KeyCode.R;

    void Update()
    {
        if (Input.GetKeyDown(key))
            resetter.ResetRagdoll();
    }
}
