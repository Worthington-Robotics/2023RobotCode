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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

public class ArmVisualizer {
    private Mechanism2d arm;
    private MechanismRoot2d root;
    private MechanismLigament2d turret;
    private MechanismLigament2d joint;
    private MechanismLigament2d end;

    public ArmVisualizer(Vector<N3> angles) {
        arm = new Mechanism2d(3, 3);
        root = arm.getRoot("Body", 1 + 0.35, 0.0);
        turret = root.append(new MechanismLigament2d("Turret", 0.9, 90, 5, new Color8Bit(Color.kRed)));
        joint = turret.append(new MechanismLigament2d("Telescoping", -Math.PI/2+ angles.get(1, 0), Rotation2d.fromRadians(angles.get(0, 0)).getDegrees(), 3, new Color8Bit(Color.kSlateGray)));
        end = joint.append(new MechanismLigament2d("End", Constants.Arm.END_LENGTH, Rotation2d.fromRadians(angles.get(2, 0)).getDegrees(), 3, new Color8Bit(Color.kSlateGray)));
        // Pose3d baseGay = new Pose3d(0.35, 1, 0.9, new Rotation3d(Math.PI/2, -Rotation2d.fromDegrees(joint.getAngle()).getRadians(), Math.PI));
        Pose3d baseGay = new Pose3d(0, 0, 0, new Rotation3d(Rotation2d.fromDegrees(joint.getAngle()).getRadians(), Math.PI, -Math.PI/2 + Math.PI));
        // Pose3d baseGay = new Pose3d(0, 0, 0, new Rotation3d(0, -Math.PI/2, 0));
        SmartDashboard.putNumberArray("baseGay", new double[] {
            baseGay.getX(), baseGay.getY(), baseGay.getZ(), baseGay.getRotation().getQuaternion().getW(), baseGay.getRotation().getQuaternion().getX(), baseGay.getRotation().getQuaternion().getY(), baseGay.getRotation().getQuaternion().getZ()
        });
        SmartDashboard.putData("ArmMech", arm);
    }

    public void update(Vector<N3> angles) {
        joint.setAngle(-90 + Rotation2d.fromRadians(angles.get(0, 0)).getDegrees());
        joint.setLength(angles.get(1, 0));
        end.setAngle(Rotation2d.fromRadians(angles.get(2, 0)));
        SmartDashboard.putData("ArmMech", arm);
    }
    
}
