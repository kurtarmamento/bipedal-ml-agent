using UnityEngine;

[DefaultExecutionOrder(1000)] // run late
public class PlanarConstraint3D : MonoBehaviour
{
    public Rigidbody rb;

    [Tooltip("World-space Z value to keep this body on (e.g., 0).")]
    public float planeZ = 0f;

    [Tooltip("If true, clamps position.z to planeZ.")]
    public bool lockPositionZ = true;

    [Tooltip("If true, removes rotation around X and Y (keeps only Z/yaw in 2D plane).")]
    public bool lockRotationXY = true;

    [Tooltip("If true, zeros velocity.z and angularVelocity.x/y to prevent drift.")]
    public bool lockVelocities = true;

    void Reset()
    {
        rb = GetComponent<Rigidbody>();
    }

    void FixedUpdate()
    {
        if (rb == null || rb.isKinematic) return;

        // 1) Position projection
        if (lockPositionZ)
        {
            Vector3 p = rb.position;
            if (!Mathf.Approximately(p.z, planeZ))
            {
                p.z = planeZ;
                rb.MovePosition(p);
            }
        }

        // 2) Rotation projection (keep only Z)
        if (lockRotationXY)
        {
            Vector3 e = rb.rotation.eulerAngles;
            // Keep only z rotation; zero x,y
            Quaternion q = Quaternion.Euler(0f, 0f, e.z);
            rb.MoveRotation(q);
        }

        // 3) Velocity projection
        if (lockVelocities)
        {
            Vector3 v = rb.linearVelocity;
            v.z = 0f;
            rb.linearVelocity = v;

            Vector3 w = rb.angularVelocity;
            w.x = 0f;
            w.y = 0f;
            rb.angularVelocity = w;
        }
    }
}
