package frc.lib.trajectory.constraint;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Translation2d;

public class VelocityLimitRegionConstraint implements TrajectoryConstraint {
    protected final Translation2d min_corner_;
    protected final Translation2d max_corner_;
    protected final double velocity_limit_;

    public VelocityLimitRegionConstraint(Translation2d min_corner, Translation2d max_corner, double velocity_limit) {
        min_corner_ = min_corner;
        max_corner_ = max_corner;
        velocity_limit_ = velocity_limit;
    }

    @Override
    public double getMaxVelocity(Pose2d pose, double curvature, double velocity) {
        final Translation2d translation = pose.getTranslation();
        if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
                translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y()) {
            return velocity_limit_;
        }
        return Double.POSITIVE_INFINITY;
    }

    @Override
    public MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double velocity) {
        return new MinMax();
    }
}
