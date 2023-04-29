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
import frc.robot.actions.manipulator.AutoStartIntaking;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class TestBarrierAuto extends StateMachineDescriptor{

    public TestBarrierAuto() {
        addSequential(new ZeroGyroAction(), 100);
        addParallel(new Action[] {new RunIntakeAction(0.1), new ArmPoseAction(ArmPose.HIGH)}, 250);
        addParallel(new Action[] {new RunIntakeAction(0.1), new PoseWaitAction()}, 1800);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);
        
        addSequential(new DriveNonblockingLineAction(-3, 0, -4.2 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
        addSequential(new ReachLineWaitAction(-4.2 * Constants.DRIVE_ENCODER_TO_METERS), 6000);
        // addSequential(new AutoTurnAction(Math.PI), 3000);

        // addSequential(new ArmPoseAction(ArmPose.INTAKE), 250);
        addParallel(new Action[] {new AutoTurnAction(Math.PI), new ArmPoseAction(ArmPose.INTAKE)}, 3000);
        addSequential(new PoseWaitAction(), 5000);
        addSequential(new DriveNonblockingLineAction(-2.0, 0, -.75 * Constants.DRIVE_ENCODER_TO_METERS, 0, Math.PI), 250);
        addParallel(new Action[]{new RunIntakeAction(Constants.INTAKE_POWER), new ReachLineWaitAction(-.75 * Constants.DRIVE_ENCODER_TO_METERS)}, 1400);//maybe a little long?
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), 250);
        // addSequential(new PoseWaitAction(), 5000);
        // addSequential(new AutoTurnAction(0), 3000);
        addParallel(new Action[] {new AutoTurnAction(0), new ArmPoseAction(ArmPose.UNSTOW)}, 3000);
        addSequential(new DriveNonblockingLineAction(5.0, 0, 5.0 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 250);
        addParallel(new Action[] {new ReachLineWaitAction(3.7 * Constants.DRIVE_ENCODER_TO_METERS), new ArmPoseAction(ArmPose.HYBRID), new RunIntakeAction(0.1)}, 1700);
        //addSequential(new ReachLineWaitAction(5 * Constants.DRIVE_ENCODER_TO_METERS),2500);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 3000);
    }
    
}
