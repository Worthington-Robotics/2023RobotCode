package frc.robot.actions.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmKinematics;
import frc.robot.subsystems.arm.ArmVisualizer;

public class ArmTestKinematics extends Action {

    @Override
    public void onStart() {
        ArmKinematics kinematics = new ArmKinematics();
        Pose2d out = kinematics.forward(VecBuilder.fill(-0.44, 0.6526, -0.8726));
        System.out.println("X: " + out.getX() + ", y: " + out.getY());
        Vector<N3> angles = kinematics.inverse(out);
        System.out.println("joint1: " + angles.get(0, 0) + ", length: " + angles.get(1,0) + ", joint2: " + angles.get(2, 0));
        ArmVisualizer viz = new ArmVisualizer(angles);
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return true;
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
    
}
