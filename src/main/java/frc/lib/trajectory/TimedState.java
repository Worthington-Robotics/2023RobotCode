package frc.lib.trajectory;

import frc.lib.geometry.IPose2d;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;

import java.util.Objects;

/**
 * Represents a time-parameterized trajectory. The trajectory contains of
 * various States that represent the pose, curvature, time elapsed, velocity,
 * and acceleration at that point.
 */
public class TimedState implements IPose2d<TimedState> {
    // The time elapsed since the beginning of the trajectory (s).
    public double time;

    // The speed at that point of the trajectory (m / s).
    public double velocity;

    // The acceleration at that point of the trajectory (m / s^2).
    public double acceleration;

    // The pose at that point of the trajectory (m x m x rad).
    public Pose2d pose;

    // The curvature at that point of the trajectory (rad / m).
    public double curvature;

    // The derivative of the curvature with respect to arc length ().
    public double dCurvatureDs;

    public TimedState() {
        pose = new Pose2d();
    }

    /**
     * Constructs a State with the specified parameters.
     *
     * @param timeSeconds                   The time elapsed since the beginning of
     *                                      the trajectory (s).
     * @param velocityMetersPerSecond       The speed at that point of the
     *                                      trajectory (m / s).
     * @param accelerationMetersPerSecondSq The acceleration at that point of the
     *                                      trajectory (m / s^2).
     * @param poseMeters                    The pose at that point of the trajectory
     *                                      (m x m x rad).
     * @param curvatureRadPerMeter          The curvature at that point of the
     *                                      trajectory (rad / m).
     */
    public TimedState(double timeSeconds, double velocityMetersPerSecond, double accelerationMetersPerSecondSq,
            Pose2d poseMeters, double curvatureRadPerMeter) {
        this.time = timeSeconds;
        this.velocity = velocityMetersPerSecond;
        this.acceleration = accelerationMetersPerSecondSq;
        this.pose = poseMeters;
        this.curvature = curvatureRadPerMeter;
        this.dCurvatureDs = 0;
    }

    /**
     * Constructs a State with the specified parameters.
     *
     * @param timeSeconds                   The time elapsed since the beginning of
     *                                      the trajectory (s).
     * @param velocityMetersPerSecond       The speed at that point of the
     *                                      trajectory (m / s).
     * @param accelerationMetersPerSecondSq The acceleration at that point of the
     *                                      trajectory (m / s^2).
     * @param poseMeters                    The pose at that point of the trajectory
     *                                      (m x m x rad).
     * @param curvatureRadPerMeter          The curvature at that point of the
     *                                      trajectory (rad / m).
     * @param dCurvatureDs                  The change in curvature at that point of 
     *                                      the trajectory ().
     */
    public TimedState(double timeSeconds, double velocityMetersPerSecond, double accelerationMetersPerSecondSq,
            Pose2d poseMeters, double curvatureRadPerMeter, double dCurvatureDs) {
        this.time = timeSeconds;
        this.velocity = velocityMetersPerSecond;
        this.acceleration = accelerationMetersPerSecondSq;
        this.pose = poseMeters;
        this.curvature = curvatureRadPerMeter;
        this.dCurvatureDs = dCurvatureDs;
    }

    /**
     * Interpolates between two States.
     *
     * @param endValue The end value for the interpolation.
     * @param i        The interpolant (fraction).
     * @return The interpolated state.
     */
    @Override
    public TimedState interpolate(TimedState endValue, double i) {

        // Find the new t value.
        final double newT = lerp(time, endValue.time, i);

        // Find the delta time between the current state and the interpolated state.
        final double deltaT = newT - time;

        // If delta time is negative, flip the order of interpolation.
        if (deltaT < 0) {
            return endValue.interpolate(this, 1 - i);
        }

        // Check whether the robot is reversing at this stage.
        final boolean reversing = velocity < 0 || Math.abs(velocity) < 1E-9 && acceleration < 0;

        // Calculate the new velocity
        // v_f = v_0 + at
        final double newV = velocity + (acceleration * deltaT);

        // Calculate the change in position.
        // delta_s = v_0 t + 0.5 at^2
        final double newS = (velocity * deltaT + 0.5 * acceleration * Math.pow(deltaT, 2)) * (reversing ? -1.0 : 1.0);

        // Return the new state. To find the new position for the new state, we need
        // to interpolate between the two endpoint poses. The fraction for
        // interpolation is the change in position (delta s) divided by the total
        // distance between the two endpoints.
        final double interpolationFrac = newS / endValue.pose.getTranslation().distance(pose.getTranslation());

        return new TimedState(newT, newV, acceleration, pose.interpolate(endValue.pose, interpolationFrac),
                lerp(curvature, endValue.curvature, interpolationFrac));
    }

    @Override
    public String toString() {
        return String.format("State(Sec: %.2f, Vel m/s: %.2f, Accel m/s/s: %.2f, Pose: %s, Curvature: %.2f)", time,
                velocity, acceleration, pose, curvature);
    }

    public String toCSV() {
        return String.format("%.2f, %.2f, %.2f, %s, %.2f", time, velocity, acceleration, pose.toCSV(), curvature);
    }

    @Override
    public int getNumFields() {
        return 4 + pose.getNumFields();
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (!(obj instanceof TimedState)) {
            return false;
        }
        TimedState state = (TimedState) obj;
        return Double.compare(state.time, time) == 0 && Double.compare(state.velocity, velocity) == 0
                && Double.compare(state.acceleration, acceleration) == 0
                && Double.compare(state.curvature, curvature) == 0 && Objects.equals(pose, state.pose);
    }

    @Override
    public int hashCode() {
        return Objects.hash(time, velocity, acceleration, pose, curvature);
    }

    /**
     * Linearly interpolates between two values.
     *
     * @param startValue The start value.
     * @param endValue   The end value.
     * @param t          The fraction for interpolation.
     * @return The interpolated value.
     */
    private static double lerp(double startValue, double endValue, double t) {
        return startValue + (endValue - startValue) * t;
    }

    @Override
    public Rotation2d getRotation() {
        return pose.getRotation();
    }

    @Override
    public double distance(TimedState other) {
        return pose.distance(other.pose);
    }

    @Override
    public Translation2d getTranslation() {
        return pose.getTranslation();
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public TimedState transformBy(Pose2d transform) {
        return new TimedState(time, velocity, acceleration, pose.transformBy(transform), curvature);
    }

    @Override
    public TimedState mirror() {
        return new TimedState(time, -velocity, -acceleration, pose.mirror(), -curvature);
    }

}
