## Code

```javascript
import Car from "../../physics/Car.js"
// THREE is assumed to be globally available or imported, e.g., in Simulator.js or via Utils.js
// If not, you might need: import * as THREE from 'three'; (adjust path if needed)

export default class MPCController {
  constructor(path) {
    this.path = path;
    this.nextIndex = 1;
    this.currentPathProgress = 0; // Spatial progress on the current path segment

    // MPC Parameters
    this.predictionHorizon = 15; // N: Number of steps to look ahead. Tuning: 10-30
    this.mpc_dt = 0.1;           // dt: Time step for MPC internal prediction (seconds). Tuning: 0.05-0.2

    // Weights for the cost function - CRUCIAL for tuning behavior.
    // These are starting points and require careful adjustment for desired performance.
    this.weights = {
      crossTrackError: 25.0,    // Weight for deviation perpendicular to the path. Higher = stronger path adherence.
      headingError: 20.0,       // Weight for difference in orientation to the path. Higher = better alignment.
      velocityError: 2.0,       // Weight for difference from target path velocity.
      steeringEffort: 50.0,     // Penalty for large steering angles (actuator physical limits/wear).
      accelerationEffort: 5.0,  // Penalty for large acceleration/deceleration inputs.
      steeringRate: 300.0,      // Penalty for rapid changes in steering. Higher = smoother steering.
      accelerationRate: 30.0    // Penalty for rapid changes in acceleration. Higher = smoother acceleration.
    };

    // Constraints from Car.js, can be fine-tuned if specific MPC behavior is desired
    this.constraints = {
      maxSteeringAngle: Car.MAX_WHEEL_ANGLE,
      maxSteeringRate: Car.MAX_STEER_SPEED, // Max rate of change for the steering *angle* (rad/s)
      maxAcceleration: Car.MAX_GAS_ACCEL,   // Max longitudinal acceleration (m/s^2)
      maxDeceleration: Car.MAX_BRAKE_DECEL // Max longitudinal deceleration (m/s^2, positive value)
    };

    // Discount factor for future costs in the prediction horizon.
    // Values closer to 1 give more weight to future states.
    this.costDiscountFactor = 0.98; // Tuning: 0.9-0.99

    // Store the previously applied optimal control actions (actual values, not normalized commands)
    this.prevOptimalSteeringAngle = 0;    // Actual steering angle (radians)
    this.prevOptimalAccelerationValue = 0;// Actual acceleration (m/s^2)
  }

  reset() {
    this.prevOptimalSteeringAngle = 0;
    this.prevOptimalAccelerationValue = 0;
    this.nextIndex = 1;
    this.currentPathProgress = 0;
  }

  replacePath(path) {
    this.path = path;
    this.nextIndex = 1;
    this.currentPathProgress = 0;
    // Optionally reset previous optimal controls if the new path is drastically different
    // this.prevOptimalSteeringAngle = 0;
    // this.prevOptimalAccelerationValue = 0;
  }

  // Predicts car's pose if it were to follow the path's velocity profile.
  // This is primarily for external use (e.g., simulator latency compensation)
  // and not for the MPC's internal prediction loop.
  predictPoseAfterTime(currentPose, predictionTime) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length < 2) return { ...currentPose };

    // Find current position on path using the car's reference point (e.g. rear axle)
    let [nextIdx, progress, _projectionPoint] = this.findNextIndex(currentPose.pos);
    let currentCarVelocity = currentPose.velocity;
    let timeRemaining = predictionTime;
    let accumulatedPose = { ...currentPose }; // Start with the current pose

    if (currentCarVelocity <= 0.01 && predictionTime > 0) return { ...currentPose };

    while (timeRemaining > 0) {
      if (nextIdx >= pathPoses.length) { // Reached end of path or beyond
        // Extrapolate based on the last known state if overshooting path
        const dt_extrapolate = timeRemaining;
        // Simple kinematic prediction: assume car continues with its current curvature and zero acceleration
        const steeringForCurvature = Math.atan(accumulatedPose.curv * Car.WHEEL_BASE);
        accumulatedPose = this.predictState(accumulatedPose, steeringForCurvature, 0, dt_extrapolate);
        timeRemaining = 0; // Stop prediction
        break;
      }

      const prevPathPose = pathPoses[nextIdx - 1];
      const nextPathPose = pathPoses[nextIdx];

      const segmentDist = nextPathPose.pos.distanceTo(prevPathPose.pos);
      if (segmentDist < 0.001) { // Skip zero-length segments
          progress = 0;
          nextIdx++;
          if (nextIdx >= pathPoses.length) break; // Path ended
          continue;
      }
      const distLeftInSegment = segmentDist * (1 - progress);

      // Use path's target velocity for this segment
      const avgPathSegmentVelocity = (prevPathPose.velocity + nextPathPose.velocity) / 2;
      // Effective velocity: car tries to match path's velocity
      const effectiveVelocity = Math.max(0.01, (currentCarVelocity + avgPathSegmentVelocity) / 2); // Ensure non-zero for division

      const timeToNextPoint = distLeftInSegment / effectiveVelocity;

      if (timeToNextPoint >= timeRemaining || nextIdx + 1 >= pathPoses.length) {
        const travelDist = effectiveVelocity * timeRemaining;
        const newProgressOnSegment = progress + (travelDist / segmentDist);
        const finalProgress = Math.clamp(newProgressOnSegment, 0, 1);

        accumulatedPose.pos = prevPathPose.pos.clone().lerp(nextPathPose.pos, finalProgress);
        accumulatedPose.rot = Math.wrapAngle(prevPathPose.rot + Math.wrapAngle(nextPathPose.rot - prevPathPose.rot) * finalProgress);
        accumulatedPose.curv = prevPathPose.curv + (nextPathPose.curv - prevPathPose.curv) * finalProgress;
        accumulatedPose.velocity = prevPathPose.velocity + (nextPathPose.velocity - prevPathPose.velocity) * finalProgress;
        timeRemaining = 0; // Prediction ends here
      } else {
        // Move to the end of the current segment
        accumulatedPose.pos = nextPathPose.pos.clone();
        accumulatedPose.rot = nextPathPose.rot;
        accumulatedPose.curv = nextPathPose.curv;
        accumulatedPose.velocity = nextPathPose.velocity;

        currentCarVelocity = nextPathPose.velocity; // Update car's velocity for next segment
        timeRemaining -= timeToNextPoint;
        progress = 0;
        nextIdx++;
      }
    }
    // dCurv and ddCurv are not critical for this external prediction, can be simplified.
    return { ...accumulatedPose, dCurv: 0, ddCurv: 0 };
  }

  // Kinematic bicycle model for MPC internal prediction
  predictState(initialState, steeringAngle, longitudinalAcceleration, dt) {
    let { pos, rot, velocity } = initialState;

    // Apply constraints to control inputs
    steeringAngle = Math.clamp(steeringAngle, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);
    if (longitudinalAcceleration > 0) {
      longitudinalAcceleration = Math.clamp(longitudinalAcceleration, 0, this.constraints.maxAcceleration);
    } else {
      longitudinalAcceleration = Math.clamp(longitudinalAcceleration, -this.constraints.maxDeceleration, 0);
    }

    // Avoid excessive rotation at near-zero speeds by using an effective velocity for rotation calculation.
    // const effectiveVelocityForRotation = Math.max(Math.abs(velocity), 0.1); // Use absolute for reverse

    const next_x = pos.x + velocity * Math.cos(rot) * dt;
    const next_y = pos.y + velocity * Math.sin(rot) * dt;
    // Note: steeringAngle here is the angle of the wheels, not the steering wheel.
    // Kinematic bicycle model: d(rot)/dt = v/L * tan(delta)
    const next_rot = Math.wrapAngle(rot + (velocity / Car.WHEEL_BASE) * Math.tan(steeringAngle) * dt);
    
    let next_velocity = velocity + longitudinalAcceleration * dt;

    // Prevent velocity from changing sign if braking to a stop, and ensure non-negative if moving forward.
    if (velocity > 0 && next_velocity < 0 && longitudinalAcceleration < 0) { // Braking while moving forward
        next_velocity = 0;
    } else if (velocity < 0 && next_velocity > 0 && longitudinalAcceleration > 0) { // "Braking" while moving backward (accelerating forward)
        next_velocity = 0;
    }
    // If car is intended to only move forward in this simulation, can add:
    // if (velocity >= 0) next_velocity = Math.max(0, next_velocity);


    return {
      pos: new THREE.Vector2(next_x, next_y),
      rot: next_rot,
      velocity: next_velocity,
    };
  }

  // Calculate cost for a given predicted state, reference state, and control inputs
  calculateCost(predictedState, referenceState, steeringInputValue, accelInputValue, prevAppliedSteeringAngle, prevAppliedAccelValue) {
    let cost = 0;

    // 1. Cross Track Error (CTE)
    const pathDir = THREE.Vector2.fromAngle(referenceState.rot); // Normalized direction of the path segment
    const carToRefPos = predictedState.pos.clone().sub(referenceState.pos);
    // CTE: perpendicular distance. Signed: (pathDir.x * carToRefPos.y - pathDir.y * carToRefPos.x)
    const cte = Math.abs(pathDir.x * carToRefPos.y - pathDir.y * carToRefPos.x); // Using absolute for cost
    cost += this.weights.crossTrackError * cte * cte;

    // 2. Heading Error (epsi)
    const headingError = Math.wrapAngle(predictedState.rot - referenceState.rot);
    cost += this.weights.headingError * headingError * headingError;

    // 3. Velocity Error
    const velocityError = predictedState.velocity - referenceState.velocity;
    cost += this.weights.velocityError * velocityError * velocityError;

    // 4. Steering Effort (cost on the steering angle itself)
    cost += this.weights.steeringEffort * steeringInputValue * steeringInputValue;

    // 5. Acceleration Effort (cost on the acceleration value itself)
    cost += this.weights.accelerationEffort * accelInputValue * accelInputValue;

    // 6. Steering Rate (change from previously *applied* steering angle)
    const steeringRate = steeringInputValue - prevAppliedSteeringAngle; // Using actual angle
    cost += this.weights.steeringRate * steeringRate * steeringRate;

    // 7. Acceleration Rate (change from previously *applied* acceleration value)
    const accelRate = accelInputValue - prevAppliedAccelValue; // Using actual m/s^2
    cost += this.weights.accelerationRate * accelRate * accelRate;

    return cost;
  }

  // Get reference state (target) from the planned path at a certain time ahead on the prediction horizon.
  // `currentCarTimeOnPath`: Estimated total time the car has already spent traversing the path up to its current position.
  // `timeAheadOnHorizon`: The lookahead time *within the current prediction horizon step* (e.g., mpc_dt, 2*mpc_dt, ... N*mpc_dt).
  getReferenceState(currentCarTimeOnPath, timeAheadOnHorizon) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length < 2) {
        console.warn("MPC: Path is too short in getReferenceState. Returning a default/static state.");
        return { pos: new THREE.Vector2(this.path.poses[0]?.pos.x || 0, this.path.poses[0]?.pos.y || 0), rot: this.path.poses[0]?.rot || 0, velocity: 0, curv: 0 };
    }

    let targetOverallTime = currentCarTimeOnPath + timeAheadOnHorizon;
    let accumulatedTimeAlongPath = 0;
    
    for (let i = 1; i < pathPoses.length; i++) {
        const p0 = pathPoses[i-1];
        const p1 = pathPoses[i];
        const segmentDist = p0.pos.distanceTo(p1.pos);
        // Use the average of the segment's start and end velocities for time estimation
        const avgSegmentVelocity = Math.max(0.01, (p0.velocity + p1.velocity) / 2); 
        const timeToTraverseSegment = segmentDist / avgSegmentVelocity;

        if (accumulatedTimeAlongPath + timeToTraverseSegment >= targetOverallTime) {
            // The target time falls within this current segment [p0, p1]
            const timeIntoThisSegment = targetOverallTime - accumulatedTimeAlongPath;
            // Progress (0-1) along this segment based on time
            const progressOnSegment = Math.clamp(timeIntoThisSegment / timeToTraverseSegment, 0, 1);
            
            return {
                pos: p0.pos.clone().lerp(p1.pos, progressOnSegment),
                rot: Math.wrapAngle(p0.rot + Math.wrapAngle(p1.rot - p0.rot) * progressOnSegment),
                velocity: p0.velocity + (p1.velocity - p0.velocity) * progressOnSegment,
                curv: p0.curv + (p1.curv - p0.curv) * progressOnSegment,
            };
        }
        accumulatedTimeAlongPath += timeToTraverseSegment;
    }

    // If targetOverallTime is beyond the total time duration of the path, return the last pose of the path.
    return { ...pathPoses[pathPoses.length - 1] };
  }

  // Optimize control inputs using a simple grid search over the first control action.
  // This first action is then assumed to be held constant over the prediction horizon for evaluation.
  optimizeControl(currentState, currentCarTimeOnPath) {
    const numSteeringSteps = 7; // e.g., -max, -mid, -low, 0, low, mid, max
    const numAccelSteps = 5;   // e.g., -max_decel, -low_decel, 0, low_accel, max_accel

    let bestTotalCost = Infinity;
    // Initialize optimal commands with previous ones to encourage smoothness if no better option is found
    let optimalSteeringAngleCmd = this.prevOptimalSteeringAngle; // Actual angle
    let optimalAccelValueCmd = this.prevOptimalAccelerationValue; // Actual m/s^2

    // Define search ranges for steering and acceleration more dynamically or centered.
    // For steering, search in a band around the previous command.
    const steerSearchHalfWidth = this.constraints.maxSteeringAngle * 0.75; // Explore a significant portion
    const steerMin = Math.clamp(this.prevOptimalSteeringAngle - steerSearchHalfWidth, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);
    const steerMax = Math.clamp(this.prevOptimalSteeringAngle + steerSearchHalfWidth, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);
    
    for (let i = 0; i < numSteeringSteps; i++) {
      let steeringCandidateAngle; // This will be the actual steering angle
      if (numSteeringSteps === 1) {
        steeringCandidateAngle = this.prevOptimalSteeringAngle;
      } else {
        steeringCandidateAngle = steerMin + (steerMax - steerMin) * (i / (numSteeringSteps - 1));
      }
      steeringCandidateAngle = Math.clamp(steeringCandidateAngle, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);

      for (let j = 0; j < numAccelSteps; j++) {
        let accelCandidateValue; // This will be the actual acceleration m/s^2
        if (numAccelSteps === 1) {
            accelCandidateValue = 0; // Maintain speed
        } else {
            // Sample from max deceleration to max acceleration
            accelCandidateValue = -this.constraints.maxDeceleration + 
                                  (this.constraints.maxDeceleration + this.constraints.maxAcceleration) * (j / (numAccelSteps - 1));
        }
        accelCandidateValue = Math.clamp(accelCandidateValue, -this.constraints.maxDeceleration, this.constraints.maxAcceleration);

        // Simulate trajectory for this pair of control candidates
        let currentSimStateInHorizon = { ...currentState }; // State for the current simulation roll-out
        let cumulativeCost = 0;
        
        // For rate costs, the "previous" for the first step of the horizon is the actual last applied control.
        // For subsequent steps *within this specific candidate's rollout*, if we assume the candidate control is held constant,
        // then the rate cost would be based on this constant candidate.
        let prevSteerForRateCost = this.prevOptimalSteeringAngle;
        let prevAccelForRateCost = this.prevOptimalAccelerationValue;

        for (let k = 0; k < this.predictionHorizon; k++) {
          const timeLookAheadInHorizon = (k + 1) * this.mpc_dt;
          const refState = this.getReferenceState(currentCarTimeOnPath, timeLookAheadInHorizon);

          // Predict next state using the *candidate* controls
          currentSimStateInHorizon = this.predictState(currentSimStateInHorizon, steeringCandidateAngle, accelCandidateValue, this.mpc_dt);

          // Calculate cost for this step.
          // steeringCandidateAngle & accelCandidateValue are the controls being tested.
          // prevSteerForRateCost & prevAccelForRateCost are what they are changing *from*.
          const stepCost = this.calculateCost(currentSimStateInHorizon, refState,
            steeringCandidateAngle, accelCandidateValue,
            prevSteerForRateCost, prevAccelForRateCost);

          cumulativeCost += stepCost * Math.pow(this.costDiscountFactor, k);
          
          // For the *next step in this same rollout*, the "previous" controls become the current candidates
          prevSteerForRateCost = steeringCandidateAngle;
          prevAccelForRateCost = accelCandidateValue;
        }

        if (cumulativeCost < bestTotalCost) {
          bestTotalCost = cumulativeCost;
          optimalSteeringAngleCmd = steeringCandidateAngle;
          optimalAccelValueCmd = accelCandidateValue;
        }
      }
    }
    return { steeringAngle: optimalSteeringAngleCmd, acceleration: optimalAccelValueCmd };
  }

  control(pose, currentWheelAngle, velocity, dt_sim) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length < 2) {
      this.prevOptimalSteeringAngle = 0; this.prevOptimalAccelerationValue = -this.constraints.maxDeceleration;
      return { gas: 0, brake: 1, steer: 0 }; // Path invalid or too short, emergency brake
    }

    const [nextIdx, progress, _projectionPoint] = this.findNextIndex(pose.pos);
    this.nextIndex = nextIdx;
    this.currentPathProgress = progress;

    // Estimate current car's "time" along the path, needed for getReferenceState
    let currentCarTimeOnPath = 0;
    for(let i=1; i < this.nextIndex; i++) { // Sum time for completed segments
        if (i >= pathPoses.length) break; // Should not happen if nextIndex is valid
        const p0 = pathPoses[i-1];
        const p1 = pathPoses[i];
        const segmentDist = p0.pos.distanceTo(p1.pos);
        const avgSegmentVelocity = Math.max(0.01, (p0.velocity + p1.velocity) / 2);
        currentCarTimeOnPath += segmentDist / avgSegmentVelocity;
    }
    // Add time for progress on current (partially traversed) segment
    if(this.nextIndex > 0 && this.nextIndex <= pathPoses.length) {
        const p0_curr = pathPoses[this.nextIndex-1];
        // Handle case where nextIndex might be pathPoses.length if at the very end
        const p1_curr_idx = Math.min(this.nextIndex, pathPoses.length - 1);
        const p1_curr = pathPoses[p1_curr_idx];
        
        const currentSegmentDist = p0_curr.pos.distanceTo(p1_curr.pos);
        if (currentSegmentDist > 0.001) {
            const avgCurrentSegmentVelocity = Math.max(0.01, (p0_curr.velocity + p1_curr.velocity) / 2);
            currentCarTimeOnPath += (currentSegmentDist * this.currentPathProgress) / avgCurrentSegmentVelocity;
        }
    }

    let gas = 0;
    let brake = 0;
    let steerCmdNormalized = 0; // Normalized steer command [-1, 1] for the car model

    if (this.nextIndex >= pathPoses.length && this.currentPathProgress >= 0.99) { // Stricter end condition
      brake = 1;
      this.prevOptimalSteeringAngle = 0;
      this.prevOptimalAccelerationValue = -this.constraints.maxDeceleration; // Full braking
    } else {
      const currentState = {
        pos: pose.pos.clone(),
        rot: pose.rot,
        velocity: velocity,
      };
      
      // OPTIONAL: Latency Compensation for input state to MPC
      // If `averagePlanTime` (from simulator) is significant:
      // let effectiveCurrentState = this.predictState(currentState, this.prevOptimalSteeringAngle, this.prevOptimalAccelerationValue, averagePlanTime);
      // const { steeringAngle, acceleration } = this.optimizeControl(effectiveCurrentState, currentCarTimeOnPath + averagePlanTime);
      // Else (no latency compensation):
      const { steeringAngle, acceleration } = this.optimizeControl(currentState, currentCarTimeOnPath);

      this.prevOptimalSteeringAngle = steeringAngle; // Store actual angle
      this.prevOptimalAccelerationValue = acceleration; // Store actual m/s^2

      if (acceleration > 0.01) {
        gas = Math.min(acceleration / this.constraints.maxAcceleration, 1);
        brake = 0;
      } else if (acceleration < -0.01) {
        gas = 0;
        brake = Math.min(Math.abs(acceleration) / this.constraints.maxDeceleration, 1);
      } else {
        gas = 0;
        brake = 0;
      }

      const steeringAngleError = Math.wrapAngle(steeringAngle - currentWheelAngle);
      // Required wheel angular velocity to correct the angle error within dt_sim
      const desiredWheelAngularVelocity = steeringAngleError / dt_sim;
      // Normalize this rate by the car's maximum physical steering rate
      steerCmdNormalized = Math.clamp(desiredWheelAngularVelocity / this.constraints.maxSteeringRate, -1, 1);
    }

    return { gas, brake, steer: steerCmdNormalized };
  }

  findNextIndex(currentCarPos) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length === 0) return [0, 0, new THREE.Vector2()];
    if (pathPoses.length === 1) return [1, 0, pathPoses[0].pos.clone()];

    const lookBehind = 10; 
    const lookAhead = 20;  
    const searchStart = Math.max(0, this.nextIndex - lookBehind -1); 
    const searchEnd = Math.min(pathPoses.length - 1, this.nextIndex + lookAhead);

    let closestDistSqr = Infinity;
    let closestPointOverallIdx = searchStart;

    for (let i = searchStart; i <= searchEnd; i++) {
      const distSqr = currentCarPos.distanceToSquared(pathPoses[i].pos);
      if (distSqr < closestDistSqr) {
        closestDistSqr = distSqr;
        closestPointOverallIdx = i;
      }
    }

    let projectedPoint, progress, finalNextIdx;

    if (closestPointOverallIdx === 0) {
      [projectedPoint, progress] = projectPointOnSegment(currentCarPos, pathPoses[0].pos, pathPoses[1].pos);
      finalNextIdx = 1; 
    } else if (closestPointOverallIdx === pathPoses.length - 1) {
      [projectedPoint, progress] = projectPointOnSegment(currentCarPos, pathPoses[closestPointOverallIdx - 1].pos, pathPoses[closestPointOverallIdx].pos);
      finalNextIdx = closestPointOverallIdx; 
    } else {
      const [projBefore, progBefore] = projectPointOnSegment(currentCarPos, pathPoses[closestPointOverallIdx - 1].pos, pathPoses[closestPointOverallIdx].pos);
      const distSqToSegBefore = currentCarPos.distanceToSquared(projBefore);

      const [projAfter, progAfter] = projectPointOnSegment(currentCarPos, pathPoses[closestPointOverallIdx].pos, pathPoses[closestPointOverallIdx + 1].pos);
      const distSqToSegAfter = currentCarPos.distanceToSquared(projAfter);

      if (distSqToSegBefore <= distSqToSegAfter) {
        projectedPoint = projBefore;
        progress = progBefore;
        finalNextIdx = closestPointOverallIdx;
      } else {
        projectedPoint = projAfter;
        progress = progAfter;
        finalNextIdx = closestPointOverallIdx + 1;
      }
    }
    progress = Math.clamp(progress, 0, 1);
    return [finalNextIdx, progress, projectedPoint];
  }
}

function projectPointOnSegment(point, segStart, segEnd) {
  const segmentVector = segEnd.clone().sub(segStart);
  const pointVector = point.clone().sub(segStart);
  const segmentLengthSq = segmentVector.lengthSq();

  if (segmentLengthSq < 0.000001) {
    return [segStart.clone(), 0];
  }
  const progress = pointVector.dot(segmentVector) / segmentLengthSq;
  // Note: segmentVector is modified by multiplyScalar if not cloned before.
  const projection = segStart.clone().add(segmentVector.clone().multiplyScalar(progress)); 

  return [projection, progress];
}
```

## Detailed Explanation

The `MPCController` (Model Predictive Control) is an advanced control strategy that optimizes vehicle actions by predicting future states based on a vehicle model and minimizing a predefined cost function over a finite time horizon.

### Core Logic and Principles

1.  **Model-Based Prediction (`predictState`):**
    *   At its heart, MPC uses a mathematical model of the vehicle's dynamics to predict how the car will behave in response to control inputs (steering angle, longitudinal acceleration).
    *   The provided `predictState` function implements a *kinematic bicycle model*. Given an initial state (position `pos`, rotation `rot`, `velocity`), a steering angle, a longitudinal acceleration, and a time step `dt`, it calculates the vehicle's state after `dt`.
    *   Equations used:
        *   `next_x = pos.x + velocity * cos(rot) * dt`
        *   `next_y = pos.y + velocity * sin(rot) * dt`
        *   `next_rot = wrapAngle(rot + (velocity / L) * tan(steeringAngle) * dt)` (where `L` is `Car.WHEEL_BASE`)
        *   `next_velocity = velocity + longitudinalAcceleration * dt`
    *   Control inputs are clamped to the car's physical limits (`this.constraints`).

2.  **Finite Prediction Horizon:**
    *   MPC doesn't just look at the current error; it "looks ahead" for a certain number of time steps, defined by `this.predictionHorizon` (N), with each step being `this.mpc_dt` long.
    *   For each step in this horizon, it predicts the car's state.

3.  **Cost Function (`calculateCost`):**
    *   A cost function quantifies how "good" or "bad" a predicted trajectory is. The goal of MPC is to find control inputs that minimize this cost.
    *   The cost function is a weighted sum of several terms:
        *   **`crossTrackError` (CTE):** Penalizes deviation from the reference path. Calculated as the squared perpendicular distance from the predicted car position to the reference path.
        *   **`headingError`:** Penalizes differences between the car's predicted orientation and the reference path's orientation.
        *   **`velocityError`:** Penalizes differences between the car's predicted velocity and the reference path's target velocity.
        *   **`steeringEffort`:** Penalizes large steering angles, promoting smoother steering and reducing actuator wear.
        *   **`accelerationEffort`:** Penalizes large acceleration or deceleration values.
        *   **`steeringRate`:** Penalizes rapid changes in the steering angle command compared to the previously applied command. Promotes smoother steering transitions.
        *   **`accelerationRate`:** Penalizes rapid changes in the acceleration command. Promotes smoother acceleration/deceleration.
    *   The `this.weights` object defines the relative importance of each of these terms. Tuning these weights is crucial for achieving desired driving behavior.

4.  **Optimization (`optimizeControl`):**
    *   At each control cycle, MPC attempts to find a sequence of control inputs (steering, acceleration) over the prediction horizon that minimizes the total discounted cost.
    *   The provided implementation uses a simple *grid search* (or sampling) method for the *first* control action:
        *   It discretizes the possible steering angles and acceleration values into a grid (e.g., `numSteeringSteps` x `numAccelSteps`).
        *   For each pair of candidate (steering, acceleration) actions:
            *   It assumes this first action is applied, and then (in this simplified version) held constant or a simple policy is followed for the rest of the horizon. A more complex MPC would optimize a sequence of N actions.
            *   It simulates the car's trajectory over the `predictionHorizon` using `predictState`.
            *   For each step `k` in the horizon:
                *   It gets a `referenceState` from the global path corresponding to time `currentCarTimeOnPath + (k+1)*mpc_dt`.
                *   It calculates the `stepCost` using `calculateCost` based on the predicted state, reference state, and the candidate control inputs.
                *   The cost is discounted by `Math.pow(this.costDiscountFactor, k)` to give less importance to errors further in the future.
            *   The total cumulative discounted cost for this candidate control pair is calculated.
        *   The pair of (steering, acceleration) that results in the lowest total cost is chosen as the optimal *first* action.

5.  **Receding Horizon Control:**
    *   Although MPC calculates an optimal sequence (or in this case, an optimal first action based on a horizon simulation), only the *first* control action from this optimal plan is actually applied to the car.
    *   In the next control cycle, the whole process repeats: the car's new state is measured, and a new optimization is performed over a shifted horizon. This makes MPC robust to disturbances and model inaccuracies.

6.  **Path Following (`getReferenceState`, `findNextIndex`):**
    *   `findNextIndex`: Similar to the `AutonomousController`, this function localizes the car on the global `this.path` by finding the closest segment and the car's progress along it.
    *   `getReferenceState`: For each step in the MPC's prediction horizon, a target reference state (position, rotation, velocity, curvature) is fetched from the global path. This is done by estimating the total time the car would have spent to reach a certain point in the future along the path, considering the path's own velocity profile.

7.  **Control Output (`control` method):**
    *   The `control` method orchestrates the MPC cycle.
    *   It determines `currentCarTimeOnPath` (an estimate of how far along the *time dimension* of the path the car currently is).
    *   It calls `optimizeControl` to get the optimal `steeringAngle` (actual angle in radians) and `acceleration` (actual m/s²).
    *   These optimal *actual* values are stored in `this.prevOptimalSteeringAngle` and `this.prevOptimalAccelerationValue` to be used for calculating rate penalties in the next cycle.
    *   The optimal `acceleration` value is converted into normalized `gas` (0-1) and `brake` (0-1) commands suitable for the `Car` model.
    *   The optimal `steeringAngle` is converted into a normalized `steer` rate command. This involves:
        *   Calculating the `steeringAngleError` (difference between desired optimal angle and current actual wheel angle).
        *   Determining the `desiredWheelAngularVelocity` needed to correct this error within one simulation time step (`dt_sim`).
        *   Normalizing this angular velocity by `this.constraints.maxSteeringRate` to get the `steer` command.

### Key Parameters and Tuning (`this.weights`, `this.predictionHorizon`, `this.mpc_dt`, `this.costDiscountFactor`)

*   **`this.predictionHorizon` (N):**
    *   Determines how far MPC looks into the future.
    *   Longer horizon: Allows for more proactive behavior and better handling of upcoming complex maneuvers (e.g., sharp turns). Can anticipate further.
    *   Shorter horizon: Less computationally expensive, but might be more reactive and less smooth.
    *   Tuning: Typically 10-30 steps.
*   **`this.mpc_dt`:**
    *   Time step for MPC's internal predictions.
    *   Smaller `mpc_dt`: More accurate internal simulation, but increases computation (N * (real_time_horizon / `mpc_dt`) = more prediction points).
    *   Larger `mpc_dt`: Less accurate internal simulation, faster computation.
    *   The total lookahead time is `N * mpc_dt`.
    *   Tuning: Typically 0.05s to 0.2s.
*   **`this.weights`:** These are the most critical tuning parameters.
    *   `crossTrackError`: High values force tighter path adherence.
    *   `headingError`: High values ensure the car aligns quickly with the path's orientation.
    *   `velocityError`: High values prioritize matching the speed profile of the path.
    *   `steeringEffort`, `accelerationEffort`: Penalize using large control inputs. Helps prevent jerky movements and respects actuator limits.
    *   `steeringRate`, `accelerationRate`: Penalize rapid *changes* in control inputs. Crucial for smooth driving. High values lead to very smooth, possibly sluggish, changes. Low values allow faster control adjustments.
    *   Tuning these weights is an iterative process to balance path tracking accuracy, comfort (smoothness), and responsiveness.
*   **`this.costDiscountFactor`:**
    *   Values closer to 1 (e.g., 0.98, 0.99) give more importance to errors and objectives further out in the prediction horizon.
    *   Smaller values (e.g., 0.9, 0.95) make the controller more "greedy," focusing more on immediate costs.
*   **`this.constraints`:** Define the physical limits of the car, ensuring the MPC's predictions and commanded actions are realistic.

### `predictPoseAfterTime` vs. Internal `predictState`

*   **`predictState(initialState, steeringAngle, accel, dt)`:** This is the core kinematic model used *inside* the MPC's optimization loop. It predicts the car's state one `mpc_dt` step forward given specific control inputs. It's called repeatedly for each candidate control sequence over the prediction horizon.
*   **`predictPoseAfterTime(currentPose, predictionTime)`:** This method is designed for *external* use, likely by the main simulator loop (e.g., for latency compensation). It predicts where the car would be after `predictionTime` if it *attempted to follow the global path's velocity profile*, not necessarily under the MPC's optimized control. It simulates movement along the path by considering the path's own `velocity` attributes at each point.

## Comparison with Other Controllers

### vs. ManualController

*   **Control Paradigm:**
    *   `MPCController`: Predictive, optimization-based, autonomous.
    *   `ManualController`: Reactive, direct user input.
*   **Decision Making:**
    *   `MPCController`: Solves an optimization problem based on a predictive model and cost function.
    *   `ManualController`: Based on instantaneous key presses.
*   **Complexity & Computation:**
    *   `MPCController`: High complexity, computationally intensive.
    *   `ManualController`: Very low complexity, negligible computation.

### vs. AutonomousController (PID/Stanley-like)

*   **Predictive Capability:**
    *   `MPCController`: Explicitly predicts `N` steps into the future using a vehicle model. This allows it to be proactive and anticipate future path requirements (e.g., slowing down for an upcoming sharp curve).
    *   `AutonomousController`: Largely reactive, correcting current errors. Feedforward terms (path curvature, path acceleration) provide some proactiveness, but it doesn't simulate multiple future scenarios.
*   **Optimization:**
    *   `MPCController`: Formally optimizes control inputs by minimizing a cost function. This provides a systematic way to balance multiple, potentially conflicting objectives (e.g., track path tightly vs. smooth steering vs. match speed).
    *   `AutonomousController`: Uses heuristic control laws (like PID or Stanley geometry) with tuned gains. While effective, balancing objectives is done implicitly through gain tuning.
*   **Constraint Handling:**
    *   `MPCController`: Can more naturally incorporate various constraints (state constraints, input constraints, rate constraints) into the optimization problem itself (though the provided example uses clamping and grid search within limits). Advanced MPC solvers handle constraints explicitly.
    *   `AutonomousController`: Typically handles constraints by clamping the output commands.
*   **Systematic Tuning:**
    *   `MPCController`: Tuning is primarily done by adjusting weights in the cost function, which often have a more intuitive meaning related to driving objectives (e.g., "how much do I care about CTE?").
    *   `AutonomousController`: Tuning involves adjusting PID gains or geometric controller parameters, which can sometimes be less intuitive to relate to overall driving behavior.
*   **Robustness and Performance:**
    *   `MPCController`: Generally offers better performance, especially in complex scenarios with varying conditions, due to its predictive and optimizing nature. It can often achieve smoother control.
    *   `AutonomousController`: Can be very effective and computationally cheaper for many path-following tasks but might require more aggressive tuning for high-speed or highly dynamic scenarios.
*   **Computational Cost:**
    *   `MPCController`: Significantly higher due to the need to run multiple simulations for the optimization at each time step.
    *   `AutonomousController`: Lower computational cost.

In summary, MPC is a powerful and flexible control framework well-suited for autonomous driving due to its ability to predict future behavior and optimize controls according to defined objectives and constraints. It offers a more systematic approach compared to traditional PID or geometric controllers, often leading to improved performance and smoothness, but at a higher computational price.