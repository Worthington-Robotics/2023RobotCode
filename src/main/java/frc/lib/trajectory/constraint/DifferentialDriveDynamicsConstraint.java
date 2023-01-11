package frc.lib.trajectory.constraint;

import frc.lib.geometry.Pose2d;
import frc.lib.physics.DifferentialDrive;

public class DifferentialDriveDynamicsConstraint implements TrajectoryConstraint {

    protected final DifferentialDrive drive_;
    protected final double abs_voltage_limit_;

    /**
     * 
     * @param drive units should be SI / metric
     * @param abs_voltage_limit
     */
    public DifferentialDriveDynamicsConstraint(final DifferentialDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(Pose2d pose, double curvature, double velocity) {
        return drive_.getMaxAbsVelocity(curvature, abs_voltage_limit_);
    }

    @Override
    public MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double velocity) {
        //NOTE: units cancel on angular velocity.
        DifferentialDrive.MinMax min_max = drive_.getMinMaxAcceleration(
                new DifferentialDrive.ChassisState(velocity, curvature * velocity)
                ,curvature, abs_voltage_limit_);

        return new MinMax(min_max.min, min_max.max);
    }
}
