package frc.robot.subsystems.arm;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ArmVisualizer {
    public ArmVisualizer(Vector<N3> angles) {
        Mechanism2d arm = new Mechanism2d(3, 3);
        MechanismRoot2d root = arm.getRoot("Body", 0, 0.809);
        MechanismLigament2d joint = root.append(new MechanismLigament2d("Telescoping", angles.get(1, 0), Rotation2d.fromRadians(angles.get(0, 0)).getDegrees()));
        MechanismLigament2d end = joint.append(new MechanismLigament2d("End", Constants.Arm.END_LENGTH, Rotation2d.fromRadians(angles.get(2, 0)).getDegrees()));
        Pose3d baseGay = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        SmartDashboard.putNumberArray("baseGay", new double[] {
            baseGay.getX(), baseGay.getY(), baseGay.getZ(), baseGay.getRotation().getQuaternion().getW(), baseGay.getRotation().getQuaternion().getX(), baseGay.getRotation().getQuaternion().getY(), baseGay.getRotation().getQuaternion().getZ()
        });
        SmartDashboard.putData(arm);
    }
    
}
