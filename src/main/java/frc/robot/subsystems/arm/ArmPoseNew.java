package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public record ArmPoseNew(Pose2d wpipose) {
    public static enum Preset{
        HOMED(null),
        MID(new ArmPoseNew(new Pose2d(new Translation2d(Constants.Field.BASE_TO_MID_X, Constants.Field.MID_HEIGHT), new Rotation2d())));

        private ArmPoseNew pose;

        private Preset(ArmPoseNew pose) {
            this.pose = pose;
        }

        public ArmPoseNew getPose() {
            return pose;
        }

        public Pose2d getPose2d() {
            return pose.wpipose();
        }
    }
}