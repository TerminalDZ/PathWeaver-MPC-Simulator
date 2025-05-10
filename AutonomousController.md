## Code

```javascript
import Car from "../../physics/Car.js"

export default class AutonomousController {
  constructor(path) {
    this.path = path;
    this.nextIndex = 1;
    this.prevPhiError = 0; // Note: prevPhiError is declared but not used in the provided code.
    this.prevVelocity = 0;
  }

  reset() {
    this.prevVelocity = 0;
  }

  replacePath(path) {
    this.path = path;
    this.nextIndex = 1;
  }

  predictPoseAfterTime(currentPose, predictionTime) {
    const pathPoses = this.path.poses;
    // Predict based on the car's front axle, as steering is applied there.
    const frontAxlePos = Car.getFrontAxlePosition(currentPose.pos, currentPose.rot);
    let [nextIndex, progress] = this.findNextIndex(frontAxlePos);
    let currentVelocity = currentPose.velocity;

    if (currentVelocity <= 0.01 && predictionTime > 0) return currentPose; // If stopped, no change.

    let timeElapsed = 0;
    let predictedPose = {...currentPose}; // Start with current pose

    while (timeElapsed < predictionTime) {
        if (nextIndex >= pathPoses.length) { // Reached end of path
            // Extrapolate linearly if prediction time extends beyond the path
            const remainingPredTime = predictionTime - timeElapsed;
            predictedPose.pos.x += predictedPose.velocity * Math.cos(predictedPose.rot) * remainingPredTime;
            predictedPose.pos.y += predictedPose.velocity * Math.sin(predictedPose.rot) * remainingPredTime;
            // Velocity and curvature remain as per the last path point
            break;
        }

        const prevPathPose = pathPoses[nextIndex - 1];
        const nextPathPose = pathPoses[nextIndex];
        
        const segmentDist = nextPathPose.pos.distanceTo(prevPathPose.pos);
        const distLeftInSegment = segmentDist * (1 - progress);

        // Average velocity on this segment of the path for time estimation
        const avgPathVelocity = (prevPathPose.velocity + nextPathPose.velocity) / 2;
        // Effective velocity for traversing this segment part
        const effectiveVelocity = Math.max(0.01, (currentVelocity + avgPathVelocity) / 2);

        const timeToTraverseRestOfSegment = distLeftInSegment / effectiveVelocity;
        
        const timeStep = Math.min(timeToTraverseRestOfSegment, predictionTime - timeElapsed);

        if (segmentDist > 0.001) { // Avoid division by zero for zero-length segments
            const travelDistOnSegment = effectiveVelocity * timeStep;
            progress += travelDistOnSegment / segmentDist;
        }

        if (progress >= 1.0) {
            predictedPose = { // Arrived at nextPathPose
                pos: nextPathPose.pos.clone(),
                rot: nextPathPose.rot,
                curv: nextPathPose.curv,
                velocity: nextPathPose.velocity,
                dCurv: 0, // Simplified for this prediction
                ddCurv: 0 // Simplified for this prediction
            };
            currentVelocity = nextPathPose.velocity;
            nextIndex++;
            progress = 0;
        } else {
             // Interpolate pose on the current segment
            predictedPose = {
                pos: prevPathPose.pos.clone().lerp(nextPathPose.pos, progress),
                rot: Math.wrapAngle(prevPathPose.rot + Math.wrapAngle(nextPathPose.rot - prevPathPose.rot) * progress),
                curv: prevPathPose.curv + (nextPathPose.curv - prevPathPose.curv) * progress,
                velocity: prevPathPose.velocity + (nextPathPose.velocity - prevPathPose.velocity) * progress,
                dCurv: 0, // Simplified
                ddCurv: 0  // Simplified
            };
        }
        timeElapsed += timeStep;
    }
    return predictedPose;
  }

  control(pose, wheelAngle, velocity, dt) {
    const pathPoses = this.path.poses;
    // Car's front axle position is used for path tracking calculations
    const frontAxlePos = Car.getFrontAxlePosition(pose.pos, pose.rot);
    const [nextIndex, progress] = this.findNextIndex(frontAxlePos);
    this.nextIndex = nextIndex; // Update controller's state

    let gas = 0;
    let brake = 0;
    let desiredWheelAnglePhi = 0; // The desired wheel deflection (steering angle of front wheels)

    // If at the end of the path, brake
    if (nextIndex >= pathPoses.length - 1 && progress >= 0.99) { // Adjusted threshold
      gas = 0;
      brake = 1;
      desiredWheelAnglePhi = 0;
    } else {
      // --- Longitudinal Control (Gas/Brake) ---
      const kp_a = 4.0;  // Proportional gain for velocity error
      const kd_a = 0.5;  // Derivative gain for acceleration error
      const kff_a = 0.5; // Feedforward gain for path's target acceleration

      const currentActualAccel = (velocity - this.prevVelocity) / dt; // Estimate current acceleration
      
      const targetPathPose = pathPoses[nextIndex]; // Target properties from the upcoming path point
      const prevPathPose = pathPoses[nextIndex-1];

      // Interpolate target velocity based on progress along the segment
      const interpolatedTargetVelocity = prevPathPose.velocity + (targetPathPose.velocity - prevPathPose.velocity) * progress;
      // Interpolate target acceleration
      const interpolatedTargetAcceleration = prevPathPose.acceleration + (targetPathPose.acceleration - prevPathPose.acceleration) * progress;


      const velocityError = interpolatedTargetVelocity - velocity;
      const accelerationError = interpolatedTargetAcceleration - currentActualAccel;
      
      // PID-like control for acceleration
      const targetControlAcceleration = kp_a * velocityError + kd_a * accelerationError + kff_a * interpolatedTargetAcceleration;

      if (targetControlAcceleration > 0) {
        gas = Math.min(targetControlAcceleration / Car.MAX_GAS_ACCEL, 1);
      } else {
        brake = Math.min(-targetControlAcceleration / Car.MAX_BRAKE_DECEL, 1);
      }
      this.prevVelocity = velocity;

      // --- Lateral Control (Steering) ---
      // Project front axle onto the path segment between (nextIndex-1).frontPos and nextIndex.frontPos
      const segmentStartFront = pathPoses[nextIndex - 1].frontPos;
      const segmentEndFront = pathPoses[nextIndex].frontPos;
      const [closestPointOnFrontPathSegment, _segmentProgress] = projectPointOnSegment(frontAxlePos, segmentStartFront, segmentEndFront);

      // Desired heading: Interpolate heading of the path segment
      // More robust: use heading of the path at the projected point.
      // For simplicity, interpolate between path point headings.
      const prevPathPointHeading = pathPoses[nextIndex - 1].rot;
      const nextPathPointHeading = pathPoses[nextIndex].rot;
      // The desired heading at the *car's current projection* on the path segment
      const desiredHeadingOnPath = Math.wrapAngle(prevPathPointHeading + Math.wrapAngle(nextPathPointHeading - prevPathPointHeading) * progress);

      // Cross-Track Error (CTE)
      const crossTrackError = frontAxlePos.distanceTo(closestPointOnFrontPathSegment);
      // Determine sign of CTE (left/right of path)
      // Vector from segment start to frontAxlePos
      const vecToAxle = frontAxlePos.clone().sub(segmentStartFront);
      // Vector representing the path segment direction
      const segmentVec = segmentEndFront.clone().sub(segmentStartFront);
      // 2D cross product (z-component): segmentVec.x * vecToAxle.y - segmentVec.y * vecToAxle.x
      // Positive if axle is to the left of path segment, negative if to the right.
      const cteSign = Math.sign(segmentVec.x * vecToAxle.y - segmentVec.y * vecToAxle.x) || 1;


      // Heading Error
      const headingError = Math.wrapAngle(pose.rot - desiredHeadingOnPath);

      // Stanley Controller variant for steering angle (phi)
      // phi = heading_error + atan(k * cte / (velocity + epsilon))
      // The provided code uses a slightly different formulation which also includes path curvature.
      const k_cte = 4.0;   // Gain for cross-track error
      const k_gain = 0.8;  // Overall gain for the feedback term

      // Interpolated path curvature at the current progress point
      const pathCurvature = prevPathPose.curv + (targetPathPose.curv - prevPathPose.curv) * progress;

      // Feedforward term: steering angle required to match path curvature
      const feedforwardSteer = Math.atan(pathCurvature * Car.WHEEL_BASE);
      
      // Feedback term: corrects for cross-track error
      // The original code's `dir` seems to be `cteSign`.
      // `Math.atan(k * dir * crossTrackError / Math.max(velocity, 0.01))`
      // This term aims to point the wheels towards the path.
      const feedbackSteerCTE = k_gain * Math.atan(k_cte * cteSign * crossTrackError / Math.max(Math.abs(velocity), 0.5)); // Epsilon to prevent division by zero and reduce sensitivity at low speeds

      // The original formula also implicitly includes a heading error component by calculating phi directly.
      // Let's use a more standard Stanley-like combination:
      // desiredWheelAnglePhi = headingError + feedbackSteerCTE; // Classic Stanley
      // Or, incorporating the path curvature feedforward directly:
      desiredWheelAnglePhi = feedforwardSteer + headingError + feedbackSteerCTE;
      // The original line was: `phi = Math.atan(curv * Car.WHEEL_BASE) + gain * Math.atan(k * dir * crossTrackError / Math.max(velocity, 0.01));`
      // This combines feedforward (curvature) and CTE feedback. It lacks an explicit heading error term in this form,
      // but the geometry might implicitly handle some heading alignment.
      // Let's stick to the provided logic:
      // desiredWheelAnglePhi = feedforwardSteer + k_gain * Math.atan(k_cte * cteSign * crossTrackError / Math.max(Math.abs(velocity), 0.5));
      // To match the original code more closely:
      desiredWheelAnglePhi = feedforwardSteer + k_gain * Math.atan(k_cte * cteSign * crossTrackError / Math.max(Math.abs(velocity), 0.5));
      // The original formula for phi did not explicitly sum headingError, it seems the geometric nature
      // of CTE and target curvature was intended to correct heading.
      // The `desiredHeading` was calculated but not directly used in the original phi.
      // Re-instating the original logic for phi as best as interpreted:
      // const originalPhi = Math.atan(pathCurvature * Car.WHEEL_BASE) + k_gain * Math.atan(k_cte * cteSign * crossTrackError / Math.max(Math.abs(velocity), 0.01));
      // desiredWheelAnglePhi = originalPhi;

      // A common Stanley controller formulation would be:
      desiredWheelAnglePhi = headingError + Math.atan(k_cte * cteSign * crossTrackError / (Math.abs(velocity) + 0.1)); // Adding epsilon for stability
      desiredWheelAnglePhi = Math.wrapAngle(desiredWheelAnglePhi + feedforwardSteer); // Add curvature feedforward

    }

    // Convert desired wheel angle (phi) to a steering command (rate)
    // The steering command for Car.js is a normalized rate.
    const wheelAngleError = Math.wrapAngle(desiredWheelAnglePhi - wheelAngle);
    // Required angular velocity of the steering wheel to correct the error in one time step (dt)
    const desiredSteeringRate = wheelAngleError / dt;
    // Normalize this rate by the car's maximum steering rate
    const steer = Math.clamp(desiredSteeringRate / Car.MAX_STEER_SPEED, -1, 1);

    return { gas, brake, steer };
  }

  // Finds the next point on the path the vehicle's front axle is approaching 
  // and the progress along the segment to that point.
  // Returns [nextPointIndexOnPath, progressAlongSegmentToNextPoint {0 - 1}]
  findNextIndex(currentFrontAxlePos) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length < 2) return [1,0]; // Default if path is too short

    const searchWindowStart = Math.max(0, this.nextIndex - 20);
    const searchWindowEnd = Math.min(pathPoses.length - 1, this.nextIndex + 20);

    let closestOverallPointIndex = searchWindowStart;
    let minDistanceSq = Infinity;

    // 1. Find the closest path *point* (vertex) to the car's front axle within the search window.
    for (let i = searchWindowStart; i <= searchWindowEnd; i++) {
      const distSq = currentFrontAxlePos.distanceToSquared(pathPoses[i].frontPos);
      if (distSq < minDistanceSq) {
        minDistanceSq = distSq;
        closestOverallPointIndex = i;
      }
    }

    // 2. Determine the target path *segment*.
    // The car is likely on the segment before or after `closestOverallPointIndex`.
    let targetSegmentNextIndex;
    let progressOnSegment;

    if (closestOverallPointIndex === 0) {
      // Closest to the very first point of the path, so car is on segment [0, 1]
      targetSegmentNextIndex = 1;
      progressOnSegment = projectPointOnSegment(currentFrontAxlePos, pathPoses[0].frontPos, pathPoses[1].frontPos)[1];
    } else if (closestOverallPointIndex === pathPoses.length - 1) {
      // Closest to the very last point, so car is on segment [L-2, L-1]
      targetSegmentNextIndex = pathPoses.length - 1;
      progressOnSegment = projectPointOnSegment(currentFrontAxlePos, pathPoses[pathPoses.length - 2].frontPos, pathPoses[pathPoses.length - 1].frontPos)[1];
    } else {
      // Car is somewhere in the middle. Project onto the segment before and the segment after `closestOverallPointIndex`.
      const [_, progBefore] = projectPointOnSegment(currentFrontAxlePos, pathPoses[closestOverallPointIndex - 1].frontPos, pathPoses[closestOverallPointIndex].frontPos);
      const distToSegBeforeSq = currentFrontAxlePos.distanceToSquared(
          pathPoses[closestOverallPointIndex - 1].frontPos.clone().lerp(pathPoses[closestOverallPointIndex].frontPos, Math.clamp(progBefore,0,1))
      );

      const [_, progAfter] = projectPointOnSegment(currentFrontAxlePos, pathPoses[closestOverallPointIndex].frontPos, pathPoses[closestOverallPointIndex + 1].frontPos);
      const distToSegAfterSq = currentFrontAxlePos.distanceToSquared(
          pathPoses[closestOverallPointIndex].frontPos.clone().lerp(pathPoses[closestOverallPointIndex + 1].frontPos, Math.clamp(progAfter,0,1))
      );
      
      if (distToSegBeforeSq <= distToSegAfterSq) {
        targetSegmentNextIndex = closestOverallPointIndex; // Segment is [closestOverallPointIndex-1, closestOverallPointIndex]
        progressOnSegment = progBefore;
      } else {
        targetSegmentNextIndex = closestOverallPointIndex + 1; // Segment is [closestOverallPointIndex, closestOverallPointIndex+1]
        progressOnSegment = progAfter;
      }
    }
    
    // Ensure progress is clamped [0,1] as it represents progress along the identified segment.
    return [targetSegmentNextIndex, Math.clamp(progressOnSegment, 0, 1)];
  }
}

// Helper: Projects point P onto line segment AB.
// Returns [projectedPoint, progress (t value, can be <0 or >1 if P is outside segment)]
function projectPointOnSegment(p, a, b) {
  const ab = b.clone().sub(a);
  const ap = p.clone().sub(a);
  const abLenSq = ab.lengthSq();

  if (abLenSq < 0.000001) { // Segment is essentially a point
    return [a.clone(), 0];
  }
  
  let progress = ap.dot(ab) / abLenSq;
  const projectedPoint = a.clone().add(ab.multiplyScalar(progress)); // ab is modified here

  return [projectedPoint, progress];
}
```

## Detailed Explanation

The `AutonomousController` is designed to make the vehicle follow a predefined path (`this.path`). It employs a combination of feedforward and feedback control mechanisms, resembling aspects of controllers like the Stanley controller for lateral (steering) control and a PID-like approach for longitudinal (speed) control.

### Core Logic and Principles

1.  **Path Representation:** The controller receives a `Path` object, which is an array of poses (`{pos, rot, curv, velocity, acceleration, frontPos, fakePos}`). `frontPos` (front axle position) is crucial for steering calculations.
2.  **Localization on Path (`findNextIndex`):**
    *   At each control step, the controller first determines the vehicle's current position relative to the path.
    *   It uses the car's *front axle position* (`currentFrontAxlePos`) for this.
    *   It searches a window of path points around its previously known `nextIndex` to find the path point closest to the car's front axle.
    *   Then, it projects the `currentFrontAxlePos` onto the path segments immediately before and after this closest path point.
    *   The segment yielding the projection closest to the `currentFrontAxlePos` is chosen as the current segment the car is on.
    *   It returns `nextIndex` (the index of the *end point* of the current target segment) and `progress` (a value from 0 to 1 indicating how far along the car is on this segment from its start point to `pathPoses[nextIndex]`).
3.  **Longitudinal Control (Gas/Brake):**
    *   **Target Velocity & Acceleration:** The controller aims to match the velocity and acceleration profiles defined in the `Path` object. It interpolates the target velocity and acceleration from the path based on the car's current `progress` along the active path segment.
    *   **Error Calculation:**
        *   `velocityError`: Difference between the interpolated target velocity from the path and the car's current `velocity`.
        *   `accelerationError`: Difference between the interpolated target acceleration from the path and the car's estimated current actual acceleration (`currentActualAccel`).
    *   **Control Law:** A PID-like (Proportional-Integral-Derivative) control law is used to determine the `targetControlAcceleration`:
        `targetControlAcceleration = kp_a * velocityError + kd_a * accelerationError + kff_a * interpolatedTargetAcceleration;`
        *   `kp_a * velocityError`: Proportional term, reacts to current velocity error.
        *   `kd_a * accelerationError`: Derivative-like term (as it considers error in acceleration), helps dampen response and improve stability.
        *   `kff_a * interpolatedTargetAcceleration`: Feedforward term, proactively applies the acceleration specified by the path.
    *   **Output:** The `targetControlAcceleration` is then converted into normalized `gas` (0 to 1) or `brake` (0 to 1) commands by scaling it with `Car.MAX_GAS_ACCEL` or `Car.MAX_BRAKE_DECEL`.
4.  **Lateral Control (Steering):**
    *   This part resembles a geometric path tracking controller like the Stanley controller.
    *   **Cross-Track Error (CTE):** The perpendicular distance from the car's front axle (`frontAxlePos`) to its projection on the current path segment's line (using `frontPos` from path poses). The sign of the CTE indicates whether the car is to the left or right of the path.
    *   **Heading Error:** The difference between the car's current orientation (`pose.rot`) and the desired orientation on the path. The desired orientation (`desiredHeadingOnPath`) is interpolated from the path segment the car is currently tracking.
    *   **Path Curvature Feedforward:** The term `Math.atan(pathCurvature * Car.WHEEL_BASE)` calculates the steering angle needed to maintain the current path curvature at the car's current speed (assuming a simple bicycle model). This is a feedforward component.
    *   **Feedback Correction (CTE based):** The term `k_gain * Math.atan(k_cte * cteSign * crossTrackError / Math.max(Math.abs(velocity), 0.5))` corrects for the cross-track error. It effectively tries to steer the front wheels towards the path. The `Math.max(Math.abs(velocity), 0.5)` term (an epsilon) is added for numerical stability at low speeds and to reduce steering sensitivity when slow.
    *   **Combined Desired Wheel Angle (`desiredWheelAnglePhi`):** The controller combines heading error, the CTE correction term, and the path curvature feedforward term to compute the overall `desiredWheelAnglePhi`.
        `desiredWheelAnglePhi = headingError + Math.atan(k_cte * cteSign * crossTrackError / (Math.abs(velocity) + 0.1));`
        `desiredWheelAnglePhi = Math.wrapAngle(desiredWheelAnglePhi + feedforwardSteer);`
    *   **Steering Rate Command:** The difference between the `desiredWheelAnglePhi` and the car's current `wheelAngle` is calculated. This error is then divided by the simulation time step `dt` to get a desired steering rate. This rate is normalized by `Car.MAX_STEER_SPEED` to produce the final `steer` command (`-1` to `+1`).
5.  **End of Path:** If the car reaches the end of the path (`nextIndex >= pathPoses.length - 1 && progress >= 0.99`), it applies full brakes.
6.  **`predictPoseAfterTime` Method:**
    *   This method is used externally (e.g., by the simulator for latency compensation) to estimate where the car *would be* if it continued following the current path for a given `predictionTime`.
    *   It simulates the car's movement segment by segment along the path, considering the path's velocity profile.
    *   It iteratively calculates how much time it takes to traverse the remaining part of the current segment or subsequent segments, updating the car's predicted pose and velocity accordingly.
    *   If the `predictionTime` extends beyond the path, it extrapolates the car's position linearly.

### Key Parameters and Tuning

*   **Longitudinal Gains:**
    *   `kp_a` (4.0): Proportional gain for velocity control. Higher values lead to faster response to velocity errors but can cause overshoot.
    *   `kd_a` (0.5): Derivative gain for acceleration control. Helps in damping oscillations.
    *   `kff_a` (0.5): Feedforward gain for path acceleration. Determines how much the controller relies on the path's specified acceleration.
*   **Lateral Gains (Stanley-like components):**
    *   `k_cte` (4.0 in original, used in `desiredWheelAnglePhi`): Gain for cross-track error correction. Higher values make the car return to the path more aggressively.
    *   `k_gain` (0.8 in original, used in `desiredWheelAnglePhi`): An overall gain that was part of the original CTE feedback term.
    *   The effective gains for heading error and CTE are implicitly set by their direct inclusion in the `desiredWheelAnglePhi` formula.
*   **Low-speed epsilon (e.g., 0.5 or 0.1):** Added to velocity in the denominator of the CTE correction term to prevent division by zero and reduce steering twitchiness at very low speeds.

Tuning these gains is critical for achieving stable and responsive path tracking. It typically involves trial and error in the simulation environment.

### Decision-Making Process Summary

1.  **Localize:** Determine current segment and progress on path (`findNextIndex`).
2.  **Check End Condition:** If at path end, brake.
3.  **Longitudinal Control:**
    *   Calculate target velocity and acceleration from path.
    *   Calculate velocity and acceleration errors.
    *   Apply PID-like law to get `targetControlAcceleration`.
    *   Convert to `gas`/`brake` commands.
4.  **Lateral Control:**
    *   Calculate Cross-Track Error (CTE) and its sign.
    *   Calculate Heading Error.
    *   Calculate path curvature feedforward steering angle.
    *   Calculate CTE feedback steering correction.
    *   Combine errors and feedforward to get `desiredWheelAnglePhi`.
    *   Convert `desiredWheelAnglePhi` to a normalized steering rate command.
5.  **Return Controls:** Output `gas`, `brake`, `steer`.

The controller aims for a balance: the feedforward terms (path acceleration, path curvature) proactively set the controls based on the path, while the feedback terms (velocity error, acceleration error, CTE, heading error) correct for deviations.

## Comparison with Other Controllers

### vs. ManualController

*   **Control Paradigm:**
    *   `AutonomousController`: Algorithmic, goal-oriented (path following).
    *   `ManualController`: Direct user input, no intrinsic goals.
*   **Path Awareness:**
    *   `AutonomousController`: Essential; its entire logic revolves around tracking `this.path`.
    *   `ManualController`: None.
*   **Complexity:**
    *   `AutonomousController`: Significantly more complex, involving geometric calculations, error computations, and control laws.
    *   `ManualController`: Very simple, direct mapping of key presses.

### vs. MPCController (Model Predictive Control)

*   **Predictive Horizon:**
    *   `AutonomousController`: Primarily reactive or uses very short-term implicit prediction (e.g., correcting current error to meet current/next path point). The `predictPoseAfterTime` is for external use, not its core control loop's lookahead.
    *   `MPCController`: Explicitly predicts car behavior over a longer future horizon (`predictionHorizon` steps) using a dynamic model.
*   **Optimization:**
    *   `AutonomousController`: Uses predefined control laws (PID-like, Stanley-like geometric rules) with tuned gains. It doesn't perform an explicit optimization of a cost function over future states.
    *   `MPCController`: Solves an optimization problem at each step to find control inputs that minimize a cost function over the prediction horizon. This allows it to balance multiple objectives (path tracking, comfort, effort) more systematically.
*   **Constraint Handling:**
    *   `AutonomousController`: Constraints (like max steer rate) are typically applied after calculating the desired control, by clamping the output.
    *   `MPCController`: Can incorporate constraints more directly into the optimization problem (though the provided example uses a grid search that implicitly respects actuator limits by searching within them).
*   **Computational Cost:**
    *   `AutonomousController`: Moderate, involves geometric calculations and state updates.
    *   `MPCController`: High, due to repeated model predictions for multiple candidate control sequences over the horizon.
*   **Proactiveness:**
    *   `AutonomousController`: Can be proactive due to feedforward terms (using path curvature and acceleration) but is fundamentally driven by correcting current errors relative to the path.
    *   `MPCController`: Inherently proactive because it "looks ahead" by simulating future states. This can lead to smoother and more anticipatory behavior, especially in complex scenarios or when approaching sharp curves or needing to adjust speed significantly.
*   **Tuning:**
    *   `AutonomousController`: Tuned via gains (e.g., `kp_a`, `kd_a`, `k_cte`).
    *   `MPCController`: Tuned via weights in the cost function, prediction horizon length, and MPC time step. Cost function weights often provide a more intuitive way to express control objectives.

In essence, the `AutonomousController` is a solid path follower using well-established feedback/feedforward techniques. The `MPCController` offers a more advanced, optimization-based framework that can lead to superior performance and better handling of complex objectives and constraints, at the cost of higher computational demand.