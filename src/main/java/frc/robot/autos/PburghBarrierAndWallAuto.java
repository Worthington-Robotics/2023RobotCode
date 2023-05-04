package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.AutoFieldRelAction;
import frc.robot.actions.drive.DriveFlipGyroZero;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.manipulator.AutoStartIntaking;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class PburghBarrierAndWallAuto extends StateMachineDescriptor{

    public PburghBarrierAndWallAuto() {
        addSequential(new AutoStartIntaking(), 100);
        // addSequential(new AutoSetIntakeTime(4.0), 100);
        addSequential(new ZeroGyroAction(), 100);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        addSequential(new PoseWaitAction(), 5000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);
        addSequential(new AutoFieldRelAction(-2, 0, 0), 2750);
       // addSequential(new DriveNonblockingLineAction(4, 4, (-4 * Constants.DRIVE_ENCODER_TO_METERS), 0, 1), 5000);
        //addSequential(new ReachLineWaitAction(-4 * Constants.DRIVE_ENCODER_TO_METERS), 10000);
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