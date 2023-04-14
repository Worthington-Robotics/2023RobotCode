package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.AutoFieldRelAction;
import frc.robot.actions.drive.AutoTurnAction;
import frc.robot.actions.drive.DriveFlipGyroZero;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.manipulator.AutoSetIntakeTime;
import frc.robot.actions.manipulator.AutoStartIntaking;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class BarrierAndWallAuto extends StateMachineDescriptor{

    public BarrierAndWallAuto() {
       
        addSequential(new ZeroGyroAction(), 100);
        addParallel(new Action[] {new RunIntakeAction(0.1), new ArmPoseAction(ArmPose.HIGH)}, 1200);
        addParallel(new Action[] {new RunIntakeAction(0.1), new PoseWaitAction()}, 1800);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);
        addSequential(new DriveNonblockingLineAction(-2, 0, -3.5 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
        addSequential(new AutoTurnAction(Math.PI), 3000);
        addSequential(new ArmPoseAction(ArmPose.INTAKE), 250);
        addSequential(new PoseWaitAction(), 5000);
        addParallel(new Action[] {new RunIntakeAction(Constants.INTAKE_POWER), new DriveNonblockingLineAction(1, 0, 1 * Constants.DRIVE_ENCODER_TO_METERS, 0, Math.PI)}, 1000);
        
       
       
       
       
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), 250);
        // addSequential(new PoseWaitAction(), 3000);
        // addSequential(new AutoTurnAction(0), 3000);
        // addSequential(new DriveNonblockingLineAction(2, 0, 4.5 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
        // addSequential(new ReachLineWaitAction(3 * Constants.DRIVE_ENCODER_TO_METERS), 3000);
        // addSequential(new ArmPoseAction(ArmPose.HIGH), 250);
        // addSequential(new PoseWaitAction(), 2000);
        // addSequential(new AutoTurnAction(Math.PI / 20), 3000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        // addSequential(new DriveFlipGyroZero(), 200);


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
