package frc.robot.subsystems.arm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public final record ArmPose(Pose2d wpipose) {
    public static enum Preset{
        HOMED(new ArmPose(ArmKinematics.forward(Constants.Arm.ZERO_ANGLES))),
        MID_CUBE(new ArmPose(new Pose2d(new Translation2d(Constants.Field.BASE_TO_MID_X, Constants.Field.MID_CUBE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.5)))),
        MID_CONE(new ArmPose(new Pose2d(new Translation2d(Constants.Field.BASE_TO_MID_X, Constants.Field.MID_CONE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(0)))),
        HIGH_CUBE(new ArmPose(new Pose2d(new Translation2d(Constants.Field.BASE_TO_HIGH_X, Constants.Field.HIGH_CUBE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.35)))),
        HIGH_CONE(new ArmPose(new Pose2d(new Translation2d(Constants.Field.BASE_TO_HIGH_X, Constants.Field.HIGH_CONE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.1))));

        private ArmPose pose;

        private Preset(ArmPose pose) {
            this.pose = pose;
        }

        public ArmPose getPose() {
            return pose;
        }

        public Pose2d getPose2d() {
            return pose.wpipose();
        }

        public Vector<N3> getAngles() {
            return ArmKinematics.inverse(pose.wpipose());
        }
    }
}