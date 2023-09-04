package frc.robot.subsystems.arm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ArmVisualizer {
    private Mechanism2d arm;
    private MechanismRoot2d root;
    private MechanismLigament2d turret;
    private MechanismLigament2d joint;
    private MechanismLigament2d end;

    private Pose3d jointPose;
    private Pose3d extensionPose;
    private Pose3d wristPose;

    public ArmVisualizer(Vector<N3> angles) {
        arm = new Mechanism2d(3, 3);
        root = arm.getRoot("Body", 1 + 0.35, 0.0);
        turret = root.append(new MechanismLigament2d("Turret", 0.9, 90, 5, new Color8Bit(Color.kRed)));
        joint = turret.append(new MechanismLigament2d("Telescoping", angles.get(1, 0), Rotation2d.fromRadians(angles.get(0, 0)).getDegrees(), 3, new Color8Bit(Color.kSlateGray)));
        end = joint.append(new MechanismLigament2d("End", Constants.Arm.END_LENGTH, Rotation2d.fromRadians(angles.get(2, 0)).getDegrees(), 3, new Color8Bit(Color.kSlateGray)));

        Pose3d zero = new Pose3d();
        logPose3d("zeroPose", zero);

        update(angles);
        
        SmartDashboard.putData("ArmMech", arm);
    }

    public void logPose3d(String name, Pose3d pose) {
        SmartDashboard.putNumberArray(name, new double[] {
            pose.getX(), pose.getY(), pose.getZ(), pose.getRotation().getQuaternion().getW(), pose.getRotation().getQuaternion().getX(), pose.getRotation().getQuaternion().getY(), pose.getRotation().getQuaternion().getZ()
        });
    }

    public void update(Vector<N3> angles) {
        joint.setAngle(-90 + Rotation2d.fromRadians(angles.get(0, 0)).getDegrees());
        joint.setLength(angles.get(1, 0));
        end.setAngle(Rotation2d.fromRadians(angles.get(2, 0)));
        SmartDashboard.putData("ArmMech", arm);

        // jointPose = new Pose3d(0, 0, 0, new Rotation3d(Rotation2d.fromDegrees(joint.getAngle()).getRadians(), Math.PI, -Math.PI/2 + Math.PI));

        // jointPose = new Pose3d(0, 0, 0, new Rotation3d(Rotation2d.fromDegrees(-90).getRadians(), Math.PI, -Math.PI/2 + Math.PI));
        // extensionPose = jointPose.transformBy(new Transform3d(new Translation3d(0.0, 0.0, 0.6526 - joint.getLength()), new Rotation3d(0.0, 0.0, 0.0)));
        // wristPose = extensionPose.transformBy(new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));
        // Pose3d zero = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        // logPose3d("jointPose", jointPose);
        // logPose3d("extensionPose", extensionPose);
        // logPose3d("wristPose", wristPose);
        // logPose3d("zeroPose", zero);

        jointPose = new Pose3d(-0.14, 0.0, 0.94, new Rotation3d(0, -angles.get(0, 0), 0));
        extensionPose = jointPose.transformBy(new Transform3d(new Translation3d(-0.6526 + joint.getLength(), 0.0, -0.055), new Rotation3d(0.0, 0.0, 0.0)));
        wristPose = extensionPose.transformBy(new Transform3d(new Translation3d(0.54 + 0.14, 0.0, 0.0), new Rotation3d(0.0, -angles.get(2, 0), 0.0)));
        logPose3d("jointPose", jointPose);
        logPose3d("extensionPose", extensionPose);
        logPose3d("wristPose", wristPose);
    }
    
}
