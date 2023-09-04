package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants;

public class ArmKinematics {
    public Pose2d forward(Vector<N3> angles) {
        Translation2d translation =  new Translation2d(angles.get(1, 0)*Math.cos(angles.get(0, 0)) + Constants.Arm.END_LENGTH * Math.cos(angles.get(0, 0) + angles.get(2, 0)),
        angles.get(1, 0)*Math.sin(angles.get(0, 0)) + Constants.Arm.END_LENGTH * Math.sin(angles.get(0, 0) + angles.get(2, 0)));
        return new Pose2d(translation, new Rotation2d(angles.get(0, 0) + angles.get(2, 0)));
    }

    public Vector<N3> inverse(Pose2d coordinates) {
        double joint1 = Math.atan2(coordinates.getY() - Constants.Arm.END_LENGTH * Math.sin(coordinates.getRotation().getRadians()), coordinates.getX()-Constants.Arm.END_LENGTH*Math.cos(coordinates.getRotation().getRadians()));
        double length = Math.sqrt(Math.pow(coordinates.getX(), 2)+Math.pow(coordinates.getY(), 2) + Math.pow(Constants.Arm.END_LENGTH, 2) - 2*Constants.Arm.END_LENGTH*coordinates.getX()*Math.cos(coordinates.getRotation().getRadians())- 2*Constants.Arm.END_LENGTH*coordinates.getY()*Math.sin(coordinates.getRotation().getRadians()));
        double joint2 = coordinates.getRotation().getRadians()-joint1;
        return VecBuilder.fill(joint1, length, joint2);
    }

    public Vector<N3> inverseSafe(Pose2d coordinates) {
        double joint1 = Math.atan2(coordinates.getY() - Constants.Arm.END_LENGTH * Math.sin(coordinates.getRotation().getRadians()), coordinates.getX()-Constants.Arm.END_LENGTH*Math.cos(coordinates.getRotation().getRadians()));
        double length = Math.sqrt(Math.pow(coordinates.getX(), 2)+Math.pow(coordinates.getY(), 2) + Math.pow(Constants.Arm.END_LENGTH, 2) - 2*Constants.Arm.END_LENGTH*coordinates.getX()*Math.cos(coordinates.getRotation().getRadians())- 2*Constants.Arm.END_LENGTH*coordinates.getY()*Math.sin(coordinates.getRotation().getRadians()));
        if (length <= Constants.Arm.EXTENSION_MIN_METERS) {
            length = Constants.Arm.EXTENSION_MIN_METERS;
        } else if (length >= Constants.Arm.EXTENSION_MAX_METERS) {
            length = Constants.Arm.EXTENSION_MAX_METERS;
        }
        double joint2 = coordinates.getRotation().getRadians()-joint1;
        return VecBuilder.fill(joint1, length, joint2);
    }
}
