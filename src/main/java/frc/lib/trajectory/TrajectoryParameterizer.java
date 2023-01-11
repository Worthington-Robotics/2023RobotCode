/*
 * MIT License
 *
 * Copyright (c) 2020 Cole Tucker
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
* MIT License
*
* Copyright (c) 2018 Team 254
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/

package frc.lib.trajectory;

import frc.lib.geometry.*;
import java.util.ArrayList;
import java.util.List;

import javax.management.RuntimeErrorException;

import frc.lib.trajectory.constraint.TrajectoryConstraint;
import frc.lib.trajectory.constraint.TrajectoryConstraint.MinMax;


public final class TrajectoryParameterizer{

    public static final double kEpsilon = 1e-6;

    private TrajectoryParameterizer() {
    }

    /**
     * Parameterize the trajectory by time. This is where the velocity profile is
     * generated.
     *
     * <p>The derivation of the algorithm used can be found
     * <a href="http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf">
     * here</a>.
     *
     * @param points                           Reference to the spline points.
     * @param constraints                      A vector of various velocity and acceleration.
     *                                         constraints.
     * @param startVelocityMetersPerSecond     The start velocity for the trajectory.
     * @param endVelocityMetersPerSecond       The end velocity for the trajectory.
     * @param maxVelocityMetersPerSecond       The max velocity for the trajectory.
     * @param maxAccelerationMetersPerSecondSq The max acceleration for the trajectory.
     * @param reversed                         Whether the robot should move backwards.
     *                                         Note that the robot will still move from
     *                                         a -&gt; b -&gt; ... -&gt; z as defined in the
     *                                         waypoints.
     * @return The trajectory.
     */
    public static Trajectory timeParameterizeTrajectory(
        List<Pose2dWithCurvature> points, List<TrajectoryConstraint> constraints, double startVelocityMetersPerSecond,
        double endVelocityMetersPerSecond, double maxVelocityMetersPerSecond, double maxAccelerationMetersPerSecondSq,
        boolean reversed) {
        List<ConstrainedState> constrainedStates = new ArrayList<ConstrainedState>(points.size());

        // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
        // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
        // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
        // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
        // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).

        //parameterize the beginning state
        ConstrainedState predecessor = new ConstrainedState(points.get(0), 0, startVelocityMetersPerSecond,
            -maxAccelerationMetersPerSecondSq, maxAccelerationMetersPerSecondSq);

        //Forward pass
        for (int i = 0; i < points.size(); i++) {
            constrainedStates.add(new ConstrainedState());
            ConstrainedState constrainedState = constrainedStates.get(i);
            constrainedState.pose = points.get(i);

            // Begin constraining based on predecessor.
            double ds = constrainedState.pose.getTranslation().distance(
                predecessor.pose.getTranslation());
            constrainedState.distance = predecessor.distance + ds;

            // We may need to iterate to find the maximum end velocity and common
            // acceleration, since acceleration limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global
                // acceleration limit. vf = std::sqrt(vi^2 + 2*a*d).
                constrainedState.maxVelocity = Math.min(maxVelocityMetersPerSecond,
                    Math.sqrt(predecessor.maxVelocity
                        * predecessor.maxVelocity
                        + predecessor.maxAcceleration * ds * 2.0
                ));
                if (Double.isNaN(constrainedState.maxVelocity)) {
                    throw new RuntimeException();
                }

                // Enforce global max absolute acceleration.
                constrainedState.minAcceleration = -maxAccelerationMetersPerSecondSq;
                constrainedState.maxAcceleration = maxAccelerationMetersPerSecondSq;
                
                // At this point, the constrained state is fully constructed apart from
                // all the custom-defined user constraints.
                for (final TrajectoryConstraint constraint : constraints) {
                    //TODO all points are constrained to super small values .15ish that is wrong
                    constrainedState.maxVelocity = Math.min(
                        constrainedState.maxVelocity,
                        constraint.getMaxVelocity(
                            constrainedState.pose.getPose(), constrainedState.pose.getCurvature(),
                            constrainedState.maxVelocity)
                    );
                }

                // Now enforce all acceleration limits.
                enforceAccelerationLimits(reversed, constraints, constrainedState);

                if (ds < kEpsilon) {
                    break;
                }

                // If the actual acceleration for this state is higher than the max
                // acceleration that we applied, then we need to reduce the max
                // acceleration of the predecessor and try again.
                double actualAcceleration = (constrainedState.maxVelocity * constrainedState.maxVelocity
                - predecessor.maxVelocity * predecessor.maxVelocity)/ (ds * 2.0);

                // If we violate the max acceleration constraint, let's modify the
                // predecessor.
                if (constrainedState.maxAcceleration < actualAcceleration - kEpsilon) {
                    predecessor.maxAcceleration = constrainedState.maxAcceleration;
                } else {
                    // Constrain the predecessor's max acceleration to the current
                    // acceleration.
                    if (actualAcceleration > predecessor.minAcceleration) {
                        predecessor.maxAcceleration = actualAcceleration;
                    }
                    // If the actual acceleration is less than the predecessor's min
                    // acceleration, it will be repaired in the backward pass.
                    break;
                }
            }
            predecessor = constrainedState;
        }

        ConstrainedState successor = new ConstrainedState(points.get(points.size() - 1),
        constrainedStates.get(constrainedStates.size() - 1).distance,
        endVelocityMetersPerSecond, -maxAccelerationMetersPerSecondSq, maxAccelerationMetersPerSecondSq);

        // Backward pass
        for (int i = points.size() - 1; i >= 0; i--) {
            ConstrainedState constrainedState = constrainedStates.get(i);
            double ds = constrainedState.distance - successor.distance; // negative

            while (true) {
                // Enforce max velocity limit (reverse)
                // vf = std::sqrt(vi^2 + 2*a*d), where vi = successor.
                double newMaxVelocity = Math.sqrt(
                    successor.maxVelocity* successor.maxVelocity + successor.minAcceleration * ds * 2.0
                );

                // No more limits to impose! This state can be finalized.
                if (newMaxVelocity >= constrainedState.maxVelocity) {
                    break;
                }

                constrainedState.maxVelocity = newMaxVelocity;

                // Check all acceleration constraints with the new max velocity.
                enforceAccelerationLimits(reversed, constraints, constrainedState);

                if (ds < kEpsilon) {
                    break;
                }

                // If the actual acceleration for this state is lower than the min
                // acceleration, then we need to lower the min acceleration of the
                // successor and try again.
                double actualAcceleration = (constrainedState.maxVelocity * constrainedState.maxVelocity
                    - successor.maxVelocity* successor.maxVelocity) / (ds * 2.0);

                if (constrainedState.minAcceleration > actualAcceleration + 1E-6) {
                    successor.minAcceleration = constrainedState.minAcceleration;
                } else {
                    successor.minAcceleration = actualAcceleration;
                    break;
                }
            }
            successor = constrainedState;
        }

        // Now we can integrate the constrained states forward in time to obtain our
        // trajectory states.
        List<TimedState> states = new ArrayList<>(points.size());
        double timeSeconds = 0.0;
        double distanceMeters = 0.0;
        double velocityMetersPerSecond = 0.0;

        for (int i = 0; i < constrainedStates.size(); i++) {
            final ConstrainedState state = constrainedStates.get(i);
      
            // Calculate the change in position between the current state and the previous
            // state.
            double ds = state.distance - distanceMeters;
      
            // Calculate the acceleration between the current state and the previous
            // state.
            double accel = (state.maxVelocity * state.maxVelocity
                - velocityMetersPerSecond * velocityMetersPerSecond) / (ds * 2);
      
            // Calculate dt
            double dt = 0.0;
      if (i > 0) {
        states.get(i - 1).acceleration = reversed ? -accel : accel;
        if (Math.abs(accel) > 1E-6) {
          // v_f = v_0 + a * t
          dt = (state.maxVelocity - velocityMetersPerSecond) / accel;
        } else if (Math.abs(velocityMetersPerSecond) > 1E-6) {
          // delta_x = v * t
          dt = ds / velocityMetersPerSecond;
        } else {
          throw new RuntimeException(
              "Something went wrong at iteration " + i + " of time parameterization.");
        }
      }
      
            velocityMetersPerSecond = state.maxVelocity;
            distanceMeters = state.distance;
      
            timeSeconds += dt;
      
            states.add(new TimedState(
                timeSeconds,
                reversed ? -velocityMetersPerSecond : velocityMetersPerSecond,
                reversed ? -accel : accel,
                state.pose.getPose(), state.pose.getCurvature(), state.pose.getDCurvatureDs()
            ));
          }
      
          return new Trajectory(states);
    }

    private static void enforceAccelerationLimits(boolean reverse,
        List<TrajectoryConstraint> constraints, ConstrainedState state) {
        for (final var constraint : constraints) {
        double factor = reverse ? -1.0 : 1.0;
        final MinMax minMaxAccel = constraint.getMinMaxAcceleration(
            state.pose.getPose(), state.pose.getCurvature(),
            state.maxVelocity * factor);

        state.minAcceleration = Math.max(state.minAcceleration,
            reverse ? -minMaxAccel.maxAcceleration
                : minMaxAccel.minAcceleration);

        state.maxAcceleration = Math.min(state.maxAcceleration,
            reverse ? -minMaxAccel.minAcceleration
                : minMaxAccel.maxAcceleration);
        }
  }

    protected static class ConstrainedState {
        Pose2dWithCurvature pose;
        double distance; //m
        double maxVelocity; //m/s
        double minAcceleration; //m/s^2
        double maxAcceleration; //m/s^2
    
        ConstrainedState(Pose2dWithCurvature pose, double distanceMeters,
                         double maxVelocity,
                         double minAcceleration,
                         double maxAcceleration) {
          this.pose = pose;
          this.distance = distanceMeters; 
          this.maxVelocity = maxVelocity; 
          this.minAcceleration = minAcceleration;
          this.maxAcceleration = maxAcceleration;
        }
    
        ConstrainedState() {
          pose = new Pose2dWithCurvature();
        }
      }
}

