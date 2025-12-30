using UnityEngine;

public class FootContact : MonoBehaviour
{
    public string groundTag = "Ground";

    [Header("Debug")]
    [SerializeField] private bool isGrounded;   // shows in Inspector

    public bool IsGrounded => isGrounded;

    void OnCollisionEnter(Collision c)
    {
        if (c.collider.CompareTag(groundTag))
            isGrounded = true;
    }

    void OnCollisionExit(Collision c)
    {
        if (c.collider.CompareTag(groundTag))
            isGrounded = false;
    }
}
