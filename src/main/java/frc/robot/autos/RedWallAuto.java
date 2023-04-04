package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.AutoRobotRelAction;
import frc.robot.actions.drive.DriveFlipGyroZero;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class RedWallAuto extends StateMachineDescriptor{

    public RedWallAuto() {
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        addSequential(new PoseWaitAction(), 5000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);
        addSequential(new DriveNonblockingLineAction(4, 4, (-5 * Constants.DRIVE_ENCODER_TO_METERS), 0, 1), 5000);
        addSequential(new ReachLineWaitAction(-5 * Constants.DRIVE_ENCODER_TO_METERS), 10000);
        addSequential(new DriveFlipGyroZero(), 200);
        // addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 1750);
        // addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        // addSequential(new PoseWaitAction(), 5000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        // addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 1750);
        // addSequential(new DriveNonblockingLineAction(2, 0, -5 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);

    }
    
}
