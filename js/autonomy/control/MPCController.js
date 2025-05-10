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
    const effectiveVelocityForRotation = Math.max(Math.abs(velocity), 0.1); // Use absolute for reverse

    const next_x = pos.x + velocity * Math.cos(rot) * dt;
    const next_y = pos.y + velocity * Math.sin(rot) * dt;
    // Note: steeringAngle here is the angle of the wheels, not the steering wheel.
    const next_rot = Math.wrapAngle(rot + (velocity / Car.WHEEL_BASE) * Math.tan(steeringAngle) * dt); // More accurate with tan(steeringAngle)
    // Or, for small angles, (velocity / L) * steeringAngle * dt is a common approximation.
    // const next_rot = Math.wrapAngle(rot + (effectiveVelocityForRotation / Car.WHEEL_BASE) * steeringAngle * dt);

    let next_velocity = velocity + longitudinalAcceleration * dt;

    // Prevent velocity from changing sign if braking to a stop, and ensure non-negative if moving forward.
    if (velocity > 0 && next_velocity < 0 && longitudinalAcceleration < 0) { // Braking while moving forward
        next_velocity = 0;
    } else if (velocity < 0 && next_velocity > 0 && longitudinalAcceleration > 0) { // "Braking" while moving backward (accelerating forward)
        next_velocity = 0;
    }
    // If not specifically handling reverse, ensure velocity is non-negative if it was positive or zero.
    // if (velocity >= 0) next_velocity = Math.max(0, next_velocity);


    return {
      pos: new THREE.Vector2(next_x, next_y),
      rot: next_rot,
      velocity: next_velocity,
      // Curvature isn't directly part of this basic kinematic state update,
      // but could be if a more complex model or path curvature prediction was needed here.
    };
  }

  // Calculate cost for a given predicted state, reference state, and control inputs
  calculateCost(predictedState, referenceState, steeringInputValue, accelInputValue, prevAppliedSteeringAngle, prevAppliedAccelValue) {
    let cost = 0;

    // 1. Cross Track Error (CTE)
    const pathDir = THREE.Vector2.fromAngle(referenceState.rot); // Normalized direction of the path segment
    const carToRefPos = predictedState.pos.clone().sub(referenceState.pos);
    // CTE: perpendicular distance. Signed: (pathDir.x * carToRefPos.y - pathDir.y * carToRefPos.x)
    const cte = Math.abs(pathDir.x * carToRefPos.y - pathDir.y * carToRefPos.x);
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
    const steeringRate = steeringInputValue - prevAppliedSteeringAngle;
    cost += this.weights.steeringRate * steeringRate * steeringRate;

    // 7. Acceleration Rate (change from previously *applied* acceleration value)
    const accelRate = accelInputValue - prevAppliedAccelValue;
    cost += this.weights.accelerationRate * accelRate * accelRate;

    return cost;
  }

  // Get reference state (target) from the planned path at a certain time ahead
  getReferenceState(currentCarTimeOnPath, timeAheadOnHorizon) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length < 2) {
        // Fallback: Try to maintain current state or a zero-velocity state if path is invalid
        console.warn("MPC: Path is too short in getReferenceState. Returning a default state.");
        return { pos: new THREE.Vector2(0,0), rot: 0, velocity: 0, curv: 0 }; // Or use current car state
    }

    let targetTime = currentCarTimeOnPath + timeAheadOnHorizon;
    let accumulatedTime = 0;
    
    // It's often more robust to find the reference by spatial progress first, then map to time,
    // or directly interpolate based on expected time traversal of segments.
    // This implementation iterates through segments summing up traversal times.

    for (let i = 1; i < pathPoses.length; i++) {
        const p0 = pathPoses[i-1];
        const p1 = pathPoses[i];
        const segmentDist = p0.pos.distanceTo(p1.pos);
        const avgSegmentVelocity = Math.max(0.01, (p0.velocity + p1.velocity) / 2); // Avoid division by zero
        const timeToTraverseSegment = segmentDist / avgSegmentVelocity;

        if (accumulatedTime + timeToTraverseSegment >= targetTime) {
            // Target point is on this segment
            const timeOnThisSegment = targetTime - accumulatedTime;
            const progressOnSegment = Math.clamp(timeOnThisSegment / timeToTraverseSegment, 0, 1);
            
            return {
                pos: p0.pos.clone().lerp(p1.pos, progressOnSegment),
                rot: Math.wrapAngle(p0.rot + Math.wrapAngle(p1.rot - p0.rot) * progressOnSegment),
                velocity: p0.velocity + (p1.velocity - p0.velocity) * progressOnSegment,
                curv: p0.curv + (p1.curv - p0.curv) * progressOnSegment,
            };
        }
        accumulatedTime += timeToTraverseSegment;
    }

    // If targetTime is beyond the path's total time, return the last pose
    return { ...pathPoses[pathPoses.length - 1] };
  }


  // Optimize control inputs using a simple grid search
  optimizeControl(currentState, currentCarTimeOnPath) {
    // Number of steps for discretizing control inputs for search
    const numSteeringSteps = 7; // Odd number to include zero
    const numAccelSteps = 5;   // Odd number to include zero

    let bestTotalCost = Infinity;
    // Initialize optimal commands with previous ones to encourage smoothness if no better option is found
    let optimalSteeringAngleCmd = this.prevOptimalSteeringAngle;
    let optimalAccelValueCmd = this.prevOptimalAccelerationValue;

    // Define search ranges for steering and acceleration
    // Steering: search around the previous command or a wider range if prev is extreme.
    const steerSearchRadius = this.constraints.maxSteeringAngle * 0.5; // Search within 50% of max range
    const steerMin = Math.clamp(this.prevOptimalSteeringAngle - steerSearchRadius, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);
    const steerMax = Math.clamp(this.prevOptimalSteeringAngle + steerSearchRadius, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);
    
    for (let i = 0; i < numSteeringSteps; i++) {
      let steeringCandidate;
      if (numSteeringSteps === 1) {
        steeringCandidate = this.prevOptimalSteeringAngle; // Or 0 if more conservative
      } else {
        steeringCandidate = steerMin + (steerMax - steerMin) * (i / (numSteeringSteps - 1));
      }
      steeringCandidate = Math.clamp(steeringCandidate, -this.constraints.maxSteeringAngle, this.constraints.maxSteeringAngle);

      for (let j = 0; j < numAccelSteps; j++) {
        let accelCandidate;
        if (numAccelSteps === 1) {
            accelCandidate = 0; // Maintain speed
        } else {
            // Sample from max deceleration to max acceleration
            accelCandidate = -this.constraints.maxDeceleration + (this.constraints.maxDeceleration + this.constraints.maxAcceleration) * (j / (numAccelSteps - 1));
        }
        accelCandidate = Math.clamp(accelCandidate, -this.constraints.maxDeceleration, this.constraints.maxAcceleration);

        // Simulate trajectory for this pair of control candidates
        let currentSimState = { ...currentState }; // State for the current simulation roll-out
        let cumulativeCost = 0;
        // For rate costs within this simulation, use the *actual* previously applied controls as the baseline for the *first* step.
        // For subsequent steps in the horizon, the "previous" is the candidate itself if assuming constant control.
        let prevSteerInHorizon = this.prevOptimalSteeringAngle;
        let prevAccelInHorizon = this.prevOptimalAccelerationValue;

        for (let k = 0; k < this.predictionHorizon; k++) {
          // For simple MPC, assume control is constant over horizon, or use first step for rate cost.
          // Here, we are optimizing for the *first* control action.
          // The steeringCandidate and accelCandidate are the actions taken at the *beginning* of this mpc_dt interval.

          const timeLookAhead = (k + 1) * this.mpc_dt; // Time from start of horizon to end of current step k
          const refState = this.getReferenceState(currentCarTimeOnPath, timeLookAhead);

          currentSimState = this.predictState(currentSimState, steeringCandidate, accelCandidate, this.mpc_dt);

          const stepCost = this.calculateCost(currentSimState, refState,
            steeringCandidate, accelCandidate,
            prevSteerInHorizon, prevAccelInHorizon);

          cumulativeCost += stepCost * Math.pow(this.costDiscountFactor, k);
          
          // For the next step in *this specific candidate's rollout*, the "previous" controls applied
          // were the candidates themselves.
          prevSteerInHorizon = steeringCandidate;
          prevAccelInHorizon = accelCandidate;
        }

        if (cumulativeCost < bestTotalCost) {
          bestTotalCost = cumulativeCost;
          optimalSteeringAngleCmd = steeringCandidate;
          optimalAccelValueCmd = accelCandidate;
        }
      }
    }
    return { steeringAngle: optimalSteeringAngleCmd, acceleration: optimalAccelValueCmd };
  }

  control(pose, currentWheelAngle, velocity, dt_sim) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length < 2) {
      return { gas: 0, brake: 1, steer: 0 }; // Path invalid or too short, emergency brake
    }

    // Find car's current projection on the path
    const [nextIdx, progress, _projectionPoint] = this.findNextIndex(pose.pos);
    this.nextIndex = nextIdx;
    this.currentPathProgress = progress; // Spatial progress on current segment [0,1]

    // Estimate current car's "time" along the path for getReferenceState
    let currentCarTimeOnPath = 0;
    for(let i=1; i < this.nextIndex; i++) {
        const p0 = pathPoses[i-1];
        const p1 = pathPoses[i];
        const segmentDist = p0.pos.distanceTo(p1.pos);
        const avgSegmentVelocity = Math.max(0.01, (p0.velocity + p1.velocity) / 2);
        currentCarTimeOnPath += segmentDist / avgSegmentVelocity;
    }
    // Add time for progress on current segment
    if(this.nextIndex > 0 && this.nextIndex <= pathPoses.length) {
        const p0_curr = pathPoses[this.nextIndex-1];
        const p1_curr = pathPoses[this.nextIndex-1 < pathPoses.length -1 ? this.nextIndex : this.nextIndex-1]; // Handle end of path
        const currentSegmentDist = p0_curr.pos.distanceTo(p1_curr.pos);
        const avgCurrentSegmentVelocity = Math.max(0.01, (p0_curr.velocity + p1_curr.velocity) / 2);
        currentCarTimeOnPath += (currentSegmentDist * this.currentPathProgress) / avgCurrentSegmentVelocity;
    }


    let gas = 0;
    let brake = 0;
    let steerCmdNormalized = 0; // Normalized steer command [-1, 1]

    // Check if near the end of the path
    if (this.nextIndex >= pathPoses.length -1 && this.currentPathProgress >= 0.98) {
      brake = 1; // Hard brake at the very end
      this.prevOptimalSteeringAngle = 0; // Reset steering when stopping
      this.prevOptimalAccelerationValue = -this.constraints.maxDeceleration; // Assume braking
    } else {
      const currentState = {
        pos: pose.pos.clone(), // Car's reference point (e.g., rear axle center)
        rot: pose.rot,
        velocity: velocity,
      };

      // --- Latency Compensation (Optional but recommended for professionalism) ---
      // If averagePlanTime (MPC computation + communication latency) is known:
      // let averagePlanTime = ... simulator.averagePlanTime.average ... ; // Get from simulator
      // let effectiveCurrentState = { ...currentState };
      // if (averagePlanTime && averagePlanTime > 0.001) {
      //    effectiveCurrentState = this.predictState(currentState, this.prevOptimalSteeringAngle, this.prevOptimalAccelerationValue, averagePlanTime);
      // }
      // const { steeringAngle, acceleration } = this.optimizeControl(effectiveCurrentState, currentCarTimeOnPath + averagePlanTime);
      // --- End Latency Compensation ---
      
      const { steeringAngle, acceleration } = this.optimizeControl(currentState, currentCarTimeOnPath);


      // Store the chosen optimal controls (actual values) for the next iteration's rate costs
      this.prevOptimalSteeringAngle = steeringAngle;
      this.prevOptimalAccelerationValue = acceleration;

      // Convert optimized acceleration value to gas/brake commands
      if (acceleration > 0.01) { // Apply gas
        gas = Math.min(acceleration / this.constraints.maxAcceleration, 1);
        brake = 0;
      } else if (acceleration < -0.01) { // Apply brake
        gas = 0;
        brake = Math.min(-acceleration / this.constraints.maxDeceleration, 1);
      } else { // Coast or very small acceleration
        gas = 0;
        brake = 0;
      }

      // Convert desired steering *angle* to a normalized steering *rate* command
      // The steering command for Car.js is a rate, normalized by MAX_STEER_SPEED
      const steeringAngleError = Math.wrapAngle(steeringAngle - currentWheelAngle);
      // Required angular velocity to correct error in dt_sim
      const desiredWheelAngularVelocity = steeringAngleError / dt_sim;
      // Normalize this rate by the car's max steering angular velocity
      steerCmdNormalized = Math.clamp(desiredWheelAngularVelocity / this.constraints.maxSteeringRate, -1, 1);
    }

    return { gas, brake, steer: steerCmdNormalized };
  }

  // Finds the next point on the path the vehicle is approaching and its progress.
  // Returns [nextPointIndexOnPath, progressAlongSegmentToNextPoint, projectionPointOnSegment]
  findNextIndex(currentCarPos) {
    const pathPoses = this.path.poses;
    if (!pathPoses || pathPoses.length === 0) return [0, 0, new THREE.Vector2()]; // Should not happen if path is valid
    if (pathPoses.length === 1) return [1, 0, pathPoses[0].pos.clone()]; // Path with one point, "next" is effectively the point itself

    // Search window around the previously known nextIndex to improve efficiency and robustness
    const lookBehind = 10; // How many points to look behind current nextIndex
    const lookAhead = 20;  // How many points to look ahead
    // Ensure search indices are within path bounds
    const searchStart = Math.max(0, this.nextIndex - lookBehind -1); // -1 because nextIndex is 1-based for segments [i-1, i]
    const searchEnd = Math.min(pathPoses.length - 1, this.nextIndex + lookAhead);

    let closestDistSqr = Infinity;
    let closestPointOverallIdx = searchStart; // Index of the path point closest to the car

    // First, find the single path point closest to the car's current position
    for (let i = searchStart; i <= searchEnd; i++) {
      const distSqr = currentCarPos.distanceToSquared(pathPoses[i].pos);
      if (distSqr < closestDistSqr) {
        closestDistSqr = distSqr;
        closestPointOverallIdx = i;
      }
    }

    // Now, determine which segment the car is on by checking projection
    // onto segment ending at closestPointOverallIdx and segment starting at closestPointOverallIdx.
    let projectedPoint, progress, finalNextIdx;

    if (closestPointOverallIdx === 0) {
      // Car is closest to the start of the path, so it must be on the first segment
      [projectedPoint, progress] = projectPointOnSegment(currentCarPos, pathPoses[0].pos, pathPoses[1].pos);
      finalNextIdx = 1; // Segment is [0,1], so next point index is 1
    } else if (closestPointOverallIdx === pathPoses.length - 1) {
      // Car is closest to the end of the path, so it must be on the last segment
      [projectedPoint, progress] = projectPointOnSegment(currentCarPos, pathPoses[closestPointOverallIdx - 1].pos, pathPoses[closestPointOverallIdx].pos);
      finalNextIdx = closestPointOverallIdx; // Segment is [L-2,L-1], so next point index is L-1
    } else {
      // Car is somewhere in the middle. Check segment before and after closestPointOverallIdx.
      const [projBefore, progBefore] = projectPointOnSegment(currentCarPos, pathPoses[closestPointOverallIdx - 1].pos, pathPoses[closestPointOverallIdx].pos);
      const distSqToSegBefore = currentCarPos.distanceToSquared(projBefore);

      const [projAfter, progAfter] = projectPointOnSegment(currentCarPos, pathPoses[closestPointOverallIdx].pos, pathPoses[closestPointOverallIdx + 1].pos);
      const distSqToSegAfter = currentCarPos.distanceToSquared(projAfter);

      if (distSqToSegBefore <= distSqToSegAfter) {
        projectedPoint = projBefore;
        progress = progBefore;
        finalNextIdx = closestPointOverallIdx; // Segment is [closest-1, closest]
      } else {
        projectedPoint = projAfter;
        progress = progAfter;
        finalNextIdx = closestPointOverallIdx + 1; // Segment is [closest, closest+1]
      }
    }
    // Progress should be relative to the segment leading to finalNextIdx, so it's [0,1]
    progress = Math.clamp(progress, 0, 1);
    return [finalNextIdx, progress, projectedPoint];
  }
}

// Helper: Projects a point onto a line segment.
// Returns [pointOnSegment, progressAlongSegment (can be outside 0-1 before clamping)]
function projectPointOnSegment(point, segStart, segEnd) {
  const segmentVector = segEnd.clone().sub(segStart);
  const pointVector = point.clone().sub(segStart);
  const segmentLengthSq = segmentVector.lengthSq();

  if (segmentLengthSq < 0.000001) { // Segment is effectively a point
    return [segStart.clone(), 0]; // Project onto start of segment
  }

  // Calculate progress: dot(AP, AB) / dot(AB, AB)
  const progress = pointVector.dot(segmentVector) / segmentLengthSq;
  const projection = segStart.clone().add(segmentVector.multiplyScalar(progress)); // segmentVector is modified here

  return [projection, progress];
}