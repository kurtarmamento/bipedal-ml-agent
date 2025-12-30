using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class BipedAgent : Agent
{
    // Must match Behavior Parameters -> Vector Observation Size
    // This file emits exactly 15 observations when not in warmup.
    private const int OBS_SIZE = 15;

    [Header("References")]
    public RagdollResetter resetter;
    public Rigidbody torso;

    // 4 controlled joints (hips + knees)
    public HingeJoint leftHip;     // joint on L_Thigh, connected to torso
    public HingeJoint rightHip;    // joint on R_Thigh, connected to torso
    public HingeJoint leftKnee;    // joint on L_Shin, connected to L_Thigh
    public HingeJoint rightKnee;   // joint on R_Shin, connected to R_Thigh

    public FootContact leftFoot;
    public FootContact rightFoot;

    [Header("PD Control")]
    public float kp = 120f;
    public float kd = 12f;
    public float maxTorque = 120f;

    [Header("Target angle ranges (deg)")]
    public Vector2 hipRangeDeg = new Vector2(-45f, 45f);
    public Vector2 kneeRangeDeg = new Vector2(-10f, 140f);

    [Header("Termination")]
    public float minTorsoHeight = 0.75f;
    public float maxAbsTorsoAngleDeg = 60f;

    [Header("Rewards")]
    public float forwardWeight = 1.0f;     // reward per meter forward
    public float aliveReward = 0.0f;     // per physics step (only after warmup)
    public float energyWeight = 0.0002f;   // penalty scale
    public float fallPenalty = -1.0f;

    [Header("Shaping (Stage 1: stand up)")]
    public float uprightRewardWeight = 0.005f;   // start here
    public float uprightAngleDeg = 45f;          // 0 reward at/above this tilt
    public float heightRewardScale = 0.25f;      // how quickly height reward saturates

    [Header("Gait rewards")]
    public float stepTouchdownReward = 0.4f;      // reward per valid touchdown
    public float stepLengthScale = 0.6f;          // additional reward per meter of step length
    public float minAirTime = 0.08f;              // seconds; filters chatter
    public float maxTimeBetweenSteps = 1.0f;      // seconds; encourage cadence
    public float noStepPenalty = 0.05f;           // penalty per second beyond maxTimeBetweenSteps

    public float slipPenaltyWeight = 0.2f;        // penalize horizontal slip while foot grounded
    public float maxAllowedSlipSpeed = 0.2f;      // m/s before penalty grows

    [Header("Forward reward gating")]
    public float forwardUprightAngleDeg = 45f;  // upright=1 at 0°, upright=0 at 45°+
    public float forwardDxClamp = 0.05f;        // clamp per-decision dx (meters)


    Rigidbody _lfRb, _rfRb;
    bool _prevLG, _prevRG;
    float _lastLLeaveTime, _lastRLeaveTime;
    float _lastTouchdownTime;
    int _lastFoot = 0; // -1 = left, +1 = right, 0 = none



    // Internal state
    private float _prevX;

    // Action targets (deg)
    private float _tLH, _tRH, _tLK, _tRK;

    // Warmup after reset to avoid NaN hinge angles on first frames
    private int _warmupFixedSteps = 0;

    private static bool IsFinite(float x) => !float.IsNaN(x) && !float.IsInfinity(x);


    public override void Initialize()
    {
        if (torso == null) torso = GetComponent<Rigidbody>();

        _prevX = torso != null ? torso.position.x : 0f;

        // neutral-ish targets
        _tLH = 0f; _tRH = 0f;
        _tLK = 20f; _tRK = 20f;

        // Safer: FootContact is often on a child collider; get the parent Rigidbody
        _lfRb = leftFoot != null ? leftFoot.GetComponentInParent<Rigidbody>() : null;
        _rfRb = rightFoot != null ? rightFoot.GetComponentInParent<Rigidbody>() : null;

        // Initialize gait timers so first episode doesn't inherit zeros if something runs early
        _prevLG = leftFoot != null && leftFoot.IsGrounded;
        _prevRG = rightFoot != null && rightFoot.IsGrounded;
        _lastLLeaveTime = Time.time;
        _lastRLeaveTime = Time.time;
        _lastTouchdownTime = Time.time;
        _lastFoot = 0;
    }


    public override void OnEpisodeBegin()
    {
        if (resetter != null) resetter.ResetRagdoll();

        // 2–5 fixed steps is typically enough. Increase if you still see NaN angles.
        _warmupFixedSteps = 3;

        if (torso != null)
            _prevX = torso.position.x;

        // small randomization (safe-clamped)
        _tLH = Mathf.Clamp(Random.Range(hipRangeDeg.x, hipRangeDeg.y) * 0.2f, hipRangeDeg.x, hipRangeDeg.y);
        _tRH = Mathf.Clamp(Random.Range(hipRangeDeg.x, hipRangeDeg.y) * 0.2f, hipRangeDeg.x, hipRangeDeg.y);
        _tLK = Mathf.Clamp(20f + Random.Range(-5f, 5f), kneeRangeDeg.x, kneeRangeDeg.y);
        _tRK = Mathf.Clamp(20f + Random.Range(-5f, 5f), kneeRangeDeg.x, kneeRangeDeg.y);

        // cache foot rigidbodies in case references changed
        _lfRb = leftFoot != null ? leftFoot.GetComponentInParent<Rigidbody>() : null;
        _rfRb = rightFoot != null ? rightFoot.GetComponentInParent<Rigidbody>() : null;

        // initialize gait state
        _prevLG = leftFoot != null && leftFoot.IsGrounded;
        _prevRG = rightFoot != null && rightFoot.IsGrounded;

        _lastLLeaveTime = Time.time;
        _lastRLeaveTime = Time.time;
        _lastTouchdownTime = Time.time;
        _lastFoot = 0;


    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // During warmup: output exactly OBS_SIZE zeros to avoid NaNs from joints immediately after reset.
        if (_warmupFixedSteps > 0)
        {
            for (int i = 0; i < OBS_SIZE; i++) sensor.AddObservation(0f);
            return;
        }

        // Torso angle around Z (planar)
        float torsoAngleZ = 0f;
        if (torso != null)
            torsoAngleZ = Mathf.DeltaAngle(0f, torso.rotation.eulerAngles.z); // [-180,180]
        float torsoAngleNorm = IsFinite(torsoAngleZ) ? (torsoAngleZ / 180f) : 0f;
        sensor.AddObservation(torsoAngleNorm);

        // Torso linear velocity
        float vx = torso != null ? torso.linearVelocity.x : 0f;
        float vy = torso != null ? torso.linearVelocity.y : 0f;
        sensor.AddObservation(IsFinite(vx) ? vx : 0f);
        sensor.AddObservation(IsFinite(vy) ? vy : 0f);

        // Torso angular velocity around Z
        float wz = torso != null ? torso.angularVelocity.z : 0f;
        sensor.AddObservation(IsFinite(wz) ? wz : 0f);

        // Torso height
        float y = torso != null ? torso.position.y : 0f;
        sensor.AddObservation(IsFinite(y) ? y : 0f);

        // Joint observations: (normalized angle, relative omega) per joint
        AddJointObs(sensor, leftHip, hipRangeDeg);
        AddJointObs(sensor, rightHip, hipRangeDeg);
        AddJointObs(sensor, leftKnee, kneeRangeDeg);
        AddJointObs(sensor, rightKnee, kneeRangeDeg);

        // Foot contacts
        sensor.AddObservation(leftFoot != null && leftFoot.IsGrounded ? 1f : 0f);
        sensor.AddObservation(rightFoot != null && rightFoot.IsGrounded ? 1f : 0f);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // If warmup is active, ignore actions.
        if (_warmupFixedSteps > 0) return;

        var a = actions.ContinuousActions;

        _tLH = MapToRangeDeg(a[0], hipRangeDeg);
        _tRH = MapToRangeDeg(a[1], hipRangeDeg);
        _tLK = MapToRangeDeg(a[2], kneeRangeDeg);
        _tRK = MapToRangeDeg(a[3], kneeRangeDeg);

        // Forward progress reward (delta-x), gated by uprightness and clipped
        if (torso != null)
        {
            float x = torso.position.x;
            float dx = x - _prevX;
            _prevX = x;

            float angle = Mathf.Abs(Mathf.DeltaAngle(0f, torso.rotation.eulerAngles.z)); // degrees
            if (!IsFinite(angle)) angle = forwardUprightAngleDeg;

            float upright = Mathf.Clamp01(1f - angle / Mathf.Max(1e-3f, forwardUprightAngleDeg));

            float dxClipped = Mathf.Clamp(dx, -forwardDxClamp, forwardDxClamp);

            float r = forwardWeight * upright * dxClipped;
            if (IsFinite(r))
                AddReward(r);
        }
    }


    private void FixedUpdate()
    {
        // Warmup: do nothing (no torques, no rewards)
        if (_warmupFixedSteps > 0)
        {
            // Keep prevX synced with reset motion so first dx isn't polluted by teleport
            if (torso != null)
                _prevX = torso.position.x;

            _warmupFixedSteps--;
            return;
        }

        // Terminate on physics tick (important if Decision Period > 1)
        if (torso != null)
        {
            float absTorsoAngle = Mathf.Abs(Mathf.DeltaAngle(0f, torso.rotation.eulerAngles.z));
            float torsoY = torso.position.y;

            if (!IsFinite(absTorsoAngle)) absTorsoAngle = 0f;
            if (!IsFinite(torsoY)) torsoY = 0f;

            if (torsoY < minTorsoHeight || absTorsoAngle > maxAbsTorsoAngleDeg)
            {
                AddReward(fallPenalty);
                EndEpisode();
                return;
            }
        }

        // Alive reward (currently 0)
        if (IsFinite(aliveReward) && aliveReward != 0f)
            AddReward(aliveReward);

        float energy = 0f;
        energy += ApplyHingePD(leftHip, _tLH);
        energy += ApplyHingePD(rightHip, _tRH);
        energy += ApplyHingePD(leftKnee, _tLK);
        energy += ApplyHingePD(rightKnee, _tRK);

        // Upright + height shaping reward
        if (torso != null)
        {
            float angle = Mathf.Abs(Mathf.DeltaAngle(0f, torso.rotation.eulerAngles.z));
            if (!IsFinite(angle)) angle = uprightAngleDeg;

            float upright = Mathf.Clamp01(1f - (angle / Mathf.Max(1e-3f, uprightAngleDeg)));

            float h = torso.position.y;
            if (!IsFinite(h)) h = minTorsoHeight;

            float height = Mathf.Clamp01((h - minTorsoHeight) / Mathf.Max(1e-3f, heightRewardScale));

            float shaped = uprightRewardWeight * upright * height;
            if (IsFinite(shaped) && shaped != 0f)
                AddReward(shaped);
        }

        if (!IsFinite(energy)) energy = 0f;

        float penalty = -energyWeight * energy;
        if (IsFinite(penalty))
            AddReward(penalty);

        UpdateGaitRewards();
    }



    private float ApplyHingePD(HingeJoint j, float targetDeg)
    {
        if (j == null) return 0f;

        // Hinge angle can be NaN in unstable frames; bail out safely.
        float thetaDeg = j.angle;
        if (!IsFinite(thetaDeg) || !IsFinite(targetDeg))
            return 0f;

        Rigidbody rb = j.GetComponent<Rigidbody>();
        Rigidbody cb = j.connectedBody;
        if (rb == null) return 0f;

        // World hinge axis, guarded
        Vector3 axisWorld = j.transform.TransformDirection(j.axis);
        float axisMag = axisWorld.magnitude;
        if (axisMag < 1e-6f) return 0f;
        axisWorld /= axisMag;

        // PD terms
        float errDeg = Mathf.DeltaAngle(thetaDeg, targetDeg);
        float errRad = errDeg * Mathf.Deg2Rad;
        if (!IsFinite(errRad)) return 0f;

        Vector3 wA = rb.angularVelocity;
        Vector3 wB = cb != null ? cb.angularVelocity : Vector3.zero;
        float omegaRel = Vector3.Dot(wA - wB, axisWorld);
        if (!IsFinite(omegaRel)) return 0f;

        float tau = kp * errRad - kd * omegaRel;
        if (!IsFinite(tau)) return 0f;
        tau = Mathf.Clamp(tau, -maxTorque, maxTorque);

        // Apply equal/opposite torques
        rb.AddTorque(axisWorld * tau, ForceMode.Force);
        if (cb != null)
            cb.AddTorque(-axisWorld * tau, ForceMode.Force);

        // Energy proxy for penalty
        float e = Mathf.Abs(tau * omegaRel);
        return IsFinite(e) ? e : 0f;
    }

    private static float MapToRangeDeg(float a, Vector2 rangeDeg)
    {
        if (!IsFinite(a)) a = 0f;

        float t = 0.5f * (a + 1f);
        if (!IsFinite(t)) t = 0.5f;

        float v = Mathf.Lerp(rangeDeg.x, rangeDeg.y, t);
        return IsFinite(v) ? v : 0f;
    }

    private static void AddJointObs(VectorSensor sensor, HingeJoint j, Vector2 rangeDeg)
    {
        // Always add exactly 2 floats
        if (j == null)
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            return;
        }

        float theta = j.angle;
        if (!IsFinite(theta))
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            return;
        }

        // Normalize angle into [-1,1] using expected range
        float normTheta = Mathf.InverseLerp(rangeDeg.x, rangeDeg.y, theta) * 2f - 1f;
        if (!IsFinite(normTheta)) normTheta = 0f;
        normTheta = Mathf.Clamp(normTheta, -1f, 1f);

        // Relative angular velocity about hinge axis
        Rigidbody rb = j.GetComponent<Rigidbody>();
        Rigidbody cb = j.connectedBody;

        float omegaRel = 0f;
        if (rb != null)
        {
            Vector3 axisWorld = j.transform.TransformDirection(j.axis);
            float axisMag = axisWorld.magnitude;
            if (axisMag > 1e-6f)
            {
                axisWorld /= axisMag;
                Vector3 wB = cb != null ? cb.angularVelocity : Vector3.zero;
                omegaRel = Vector3.Dot(rb.angularVelocity - wB, axisWorld);
                if (!IsFinite(omegaRel)) omegaRel = 0f;
            }
        }

        sensor.AddObservation(normTheta);
        sensor.AddObservation(omegaRel);
    }

    // Heuristic mode (debug). This must match your ML-Agents version.
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var a = actionsOut.ContinuousActions;
        float t = Time.time;

        a[0] = Mathf.Sin(t);
        a[1] = -Mathf.Sin(t);
        a[2] = Mathf.Sin(t + 1.0f);
        a[3] = Mathf.Sin(t + 1.0f);
    }

    void UpdateGaitRewards()
    {
        if (leftFoot == null || rightFoot == null) return;

        bool LG = leftFoot.IsGrounded;
        bool RG = rightFoot.IsGrounded;

        // Track liftoff times
        if (_prevLG && !LG) _lastLLeaveTime = Time.time;
        if (_prevRG && !RG) _lastRLeaveTime = Time.time;

        // Touchdown events
        bool leftTD  = (!_prevLG && LG);
        bool rightTD = (!_prevRG && RG);

        // Reward touchdown only if it was in the air long enough (filters chatter)
        if (leftTD)
        {
            float air = Time.time - _lastLLeaveTime;
            if (IsFinite(air) && air >= minAirTime)
                RewardTouchdown(foot: -1);
        }
        if (rightTD)
        {
            float air = Time.time - _lastRLeaveTime;
            if (IsFinite(air) && air >= minAirTime)
                RewardTouchdown(foot: +1);
        }

        // Slip penalty (discourages skating)
        PenalizeSlip(LG, _lfRb);
        PenalizeSlip(RG, _rfRb);

        // Cadence shaping: penalize "no touchdown for too long"
        float dtNoStep = Time.time - _lastTouchdownTime;
        if (IsFinite(dtNoStep) && dtNoStep > maxTimeBetweenSteps)
        {
            // Interpret noStepPenalty as "penalty per second while overdue"
            AddReward(-noStepPenalty * Time.fixedDeltaTime);
        }

        _prevLG = LG;
        _prevRG = RG;
    }


    void RewardTouchdown(int foot)
    {
        // Encourage alternation: L then R then L ...
        bool alternated = (_lastFoot == 0) || (foot != _lastFoot);

        // Measure "step length" as forward separation between feet at touchdown
        float stepLen = 0f;
        if (_lfRb != null && _rfRb != null)
            stepLen = Mathf.Abs(_lfRb.position.x - _rfRb.position.x);

        // Optional: require some minimum step length to avoid tiny taps
        // if (stepLen < 0.05f) return;

        float r = 0f;
        if (alternated)
        {
            r += stepTouchdownReward;
            r += stepLengthScale * Mathf.Clamp(stepLen, 0f, 0.6f);
        }
        else
        {
            // small penalty for repeated same-foot touchdowns (prevents hopping on one leg)
            r -= 0.1f;
        }

        if (!float.IsNaN(r) && !float.IsInfinity(r))
            AddReward(r);

        _lastFoot = foot;
        _lastTouchdownTime = Time.time;
    }

    void PenalizeSlip(bool grounded, Rigidbody footRb)
    {
        if (!grounded || footRb == null) return;

        float slip = Mathf.Abs(footRb.linearVelocity.x); // planar: x is forward
        float excess = Mathf.Max(0f, slip - maxAllowedSlipSpeed);

        if (IsFinite(excess) && excess > 0f)
            AddReward(-slipPenaltyWeight * excess * Time.fixedDeltaTime);
    }



}
