package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.AutoRobotRelAction;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class RedWallAuto extends StateMachineDescriptor{

    public RedWallAuto() {
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        addSequential(new PoseWaitAction(), 5000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new AutoRobotRelAction(-2, 0, 0), 3000);

        // addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 1750);
        // addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        // addSequential(new PoseWaitAction(), 5000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        // addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 1750);
        //addSequential(new DriveNonblockingLineAction(4, 0, -5, 0, 0), 5000);

    }
    
}
