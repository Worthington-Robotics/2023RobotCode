package frc.lib.trajectory.constraint;

import frc.lib.geometry.Pose2d;

public class CentripetalAccelerationConstraint implements TrajectoryConstraint {
    final double mMaxCentripetalAccel;

    public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
        mMaxCentripetalAccel = max_centripetal_accel;
    }

    @Override
    public double getMaxVelocity(Pose2d pose, double curvature, double velocity) {
        return Math.sqrt(mMaxCentripetalAccel / Math.abs(curvature));
    }

    @Override
    public MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double velocity) {
        return new MinMax();
    }
}
