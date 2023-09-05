package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;

public record ArmPoseNew(Pose2d wpipose) {
    public static enum Preset{
        HOMED(null),
        MID_CUBE(new ArmPoseNew(new Pose2d(new Translation2d(Constants.Field.BASE_TO_MID_X, Constants.Field.MID_CUBE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.5)))),
        MID_CONE(new ArmPoseNew(new Pose2d(new Translation2d(Constants.Field.BASE_TO_MID_X, Constants.Field.MID_CONE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(0)))),
        HIGH_CUBE(new ArmPoseNew(new Pose2d(new Translation2d(Constants.Field.BASE_TO_HIGH_X, Constants.Field.HIGH_CUBE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.35)))),
        HIGH_CONE(new ArmPoseNew(new Pose2d(new Translation2d(Constants.Field.BASE_TO_HIGH_X, Constants.Field.HIGH_CONE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.1))));

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