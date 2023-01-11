/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.trajectory.constraint;

import frc.lib.geometry.*;

/**
 * An interface for defining user-defined velocity and acceleration constraints
 * while generating trajectories.
 */
public interface TrajectoryConstraint {

  /**
   * Returns the max velocity given the current pose and curvature.
   *
   * @param pose         The pose at the current point in the trajectory (m x m x rad).
   * @param curvature    The curvature at the current point in the trajectory (rad / m).
   * @param velocity     The velocity at the current point in the trajectory (m / s).
   * @return The absolute maximum velocity.
   */
  double getMaxVelocity(Pose2d pose, double curvature, double velocity);

  /**
   * Returns the minimum and maximum allowable acceleration for the trajectory
   * given pose, curvature, and speed.
   *
   * @param pose         The pose at the current point in the trajectory (m x m x rad).
   * @param curvature    The curvature at the current point in the trajectory (rad / m).
   * @param velocity     The velocity at the current point in the trajectory (m / s).
   * @return The min and max acceleration bounds.
   */
  MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double velocity);

  /**
   * Represents a minimum and maximum acceleration.
   */
  public class MinMax {
    public double minAcceleration = -Double.MAX_VALUE;
    public double maxAcceleration = +Double.MAX_VALUE;

    /**
     * Constructs a MinMax.
     *
     * @param minAcceleration The minimum acceleration (m / s^2).
     * @param maxAcceleration The maximum acceleration (m / s^2).
     */
    public MinMax(double minAcceleration,
                  double maxAcceleration) {
      this.minAcceleration = minAcceleration;
      this.maxAcceleration = maxAcceleration;
    }

    /**
     * Constructs a MinMax with default values.
     */
    public MinMax() {
    }
  }
}