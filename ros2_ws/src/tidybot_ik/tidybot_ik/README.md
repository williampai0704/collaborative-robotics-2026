  Here's what the response looks like for each failure case:
  
  Field: success
  IK Failure: False
  Singularity: False
  Collision: False
  ────────────────────────────────────────                         Field: executed 
  IK Failure: False
  Singularity: False
  Collision: False
  ────────────────────────────────────────
  Field: joint_positions
  IK Failure: best-effort solution
  Singularity: valid IK solution
  Collision: valid IK solution
  ────────────────────────────────────────
  Field: position_error
  IK Failure: large (e.g. 0.12m)
  Singularity: small (converged)
  Collision: small (converged)
  ────────────────────────────────────────
  Field: orientation_error
  IK Failure: large or 0.0
  Singularity: small or 0.0
  Collision: small or 0.0
  ────────────────────────────────────────
  Field: condition_number
  IK Failure: 0.0 (never computed)
  Singularity: high (e.g. 150.0)
  Collision: normal
  ────────────────────────────────────────
  Field: message
  IK Failure: "IK failed after 7 seeds: position error=..."
  Singularity: "Near singularity: condition number=150.0 > 100.0"
  Collision: "Arm collision detected: min distance=0.02m < 0.05m"

  Key thing: condition_number is only populated if IK succeeds — for IK failure it stays at the default 0.0 because the code returns before computing it.
  Similarly, collision distance is only checked after both IK and singularity pass.

  The checks run in this order and short-circuit:

  1. IK → fail? return early
  2. Singularity → fail? return early
  3. Collision → fail? return early
  4. All pass → success=True, optionally execute