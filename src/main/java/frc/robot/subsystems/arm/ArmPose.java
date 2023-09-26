package frc.robot.subsystems.arm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public final record ArmPose(Pose2d wpipose) {
    public static enum Preset{
        ZERO(new ArmPose(ArmKinematics.forward(Constants.Arm.ZERO_ANGLES))),
        TRANSIT(new ArmPose(new Pose2d(new Translation2d(0.6, -0.6), new Rotation2d(0)))),
        MID(new ArmPose(new Pose2d(new Translation2d(Constants.Field.BASE_TO_MID_X, Constants.Field.MID_CUBE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.95)))),
        HIGH(new ArmPose(new Pose2d(new Translation2d(Constants.Field.BASE_TO_HIGH_X, Constants.Field.HIGH_CUBE_HEIGHT - Constants.Field.ARM_HEIGHT), new Rotation2d(-0.95)))),
        SLIDE(new ArmPose(new Pose2d(new Translation2d(0.65, -0.55), new Rotation2d(0.25)))),
        INTAKE(new ArmPose(new Pose2d(new Translation2d(1, -0.85), new Rotation2d(-1)))),
        SHELF(new ArmPose(new Pose2d(new Translation2d(1, 0.16), new Rotation2d(-1.6))));

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