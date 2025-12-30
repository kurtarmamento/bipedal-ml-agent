using System;
using System.Collections.Generic;
using UnityEngine;

public class RagdollResetter : MonoBehaviour
{
    [Header("Assign explicitly if you want; otherwise auto-finds in children")]
    public Rigidbody torso;

    [Header("Reset Options")]
    public bool resetToSpawnTransform = true;
    public Transform spawn; // optional; if null, uses initial root pose

    // Internal cached state
    private struct BodyState
    {
        public Rigidbody rb;
        public Vector3 localPos;
        public Quaternion localRot;
        public RigidbodyConstraints constraints;
    }

    private readonly List<BodyState> _bodies = new();
    private Vector3 _rootPos0;
    private Quaternion _rootRot0;

    void Awake()
    {
        CacheInitialState();
    }

    private void CacheInitialState()
    {
        _rootPos0 = transform.position;
        _rootRot0 = transform.rotation;

        _bodies.Clear();
        var rbs = GetComponentsInChildren<Rigidbody>(includeInactive: true);
        foreach (var rb in rbs)
        {
            _bodies.Add(new BodyState
            {
                rb = rb,
                localPos = rb.transform.localPosition,
                localRot = rb.transform.localRotation,
                constraints = rb.constraints
            });
        }

        if (torso == null)
        {
            // Try to auto-detect torso by name (optional convenience)
            foreach (var b in _bodies)
            {
                if (b.rb.name.ToLower().Contains("torso"))
                {
                    torso = b.rb;
                    break;
                }
            }
        }
    }

    public void ResetRagdoll()
    {
        // A) pause physics
        foreach (var b in _bodies)
        {
            b.rb.isKinematic = true;
            b.rb.detectCollisions = false;
        }

        // B) reset transforms (DO NOT set velocity on kinematic bodies)
        if (resetToSpawnTransform)
        {
            if (spawn != null) transform.SetPositionAndRotation(spawn.position, spawn.rotation);
            else transform.SetPositionAndRotation(_rootPos0, _rootRot0);
        }

        foreach (var b in _bodies)
        {
            var t = b.rb.transform;
            t.localPosition = b.localPos;
            t.localRotation = b.localRot;
        }

        ApplyPlanarConstraintsOption1();

        // C) resume physics (NOW it's valid to zero velocities)
        foreach (var b in _bodies)
        {
            b.rb.detectCollisions = true;
            b.rb.isKinematic = false;

            b.rb.linearVelocity = Vector3.zero;
            b.rb.angularVelocity = Vector3.zero;
            b.rb.WakeUp();
        }
    }


    private void ApplyPlanarConstraintsOption1()
    {
        // Planar constraint: freeze Z position on ALL bodies
        // Plus: torso freezes world rot X and Y (but not Z), per your working setup.

        foreach (var b in _bodies)
        {
            var rb = b.rb;

            // Start from current constraints (or you can start from None)
            var c = rb.constraints;

            // Always freeze Z position
            c |= RigidbodyConstraints.FreezePositionZ;

            // Clear torso/limb rotation freezes first (we will re-add appropriately)
            c &= ~(RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationY | RigidbodyConstraints.FreezeRotationZ);

            // Torso: freeze X/Y rotations only
            if (torso != null && rb == torso)
            {
                c |= RigidbodyConstraints.FreezeRotationX;
                c |= RigidbodyConstraints.FreezeRotationY;
                // Do NOT freeze rotation Z
            }

            rb.constraints = c;
        }
    }
}
