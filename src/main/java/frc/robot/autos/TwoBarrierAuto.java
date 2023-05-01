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

public class TwoBarrierAuto extends StateMachineDescriptor{

    public TwoBarrierAuto() {
        addSequential(new ZeroGyroAction(), 100);
        addParallel(new Action[] {new RunIntakeAction(0.1), new ArmPoseAction(ArmPose.HIGH)}, 250);
        addParallel(new Action[] {new RunIntakeAction(0.1), new PoseWaitAction()}, 1800);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);
        
        addSequential(new DriveNonblockingLineAction(-3, 0, -4.2 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
        addSequential(new ReachLineWaitAction(-4.2 * Constants.DRIVE_ENCODER_TO_METERS), 6000);
   
        addParallel(new Action[] {new AutoTurnAction(Math.PI), new ArmPoseAction(ArmPose.INTAKE)}, 3000);
        addSequential(new PoseWaitAction(), 5000);
        addSequential(new DriveNonblockingLineAction(-2.0, 0, -.6 * Constants.DRIVE_ENCODER_TO_METERS, 0, Math.PI), 250);
        addParallel(new Action[]{new RunIntakeAction(Constants.INTAKE_POWER), new ReachLineWaitAction(-.6 * Constants.DRIVE_ENCODER_TO_METERS)}, 1150);//maybe a little long?
       
        addParallel(new Action[] {new AutoTurnAction(0), new ArmPoseAction(ArmPose.UNSTOW), new RunIntakeAction(0.1)}, 1800);
        addParallel(new Action[] {new DriveNonblockingLineAction(5.0, 0, 5.5 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), new RunIntakeAction(0.1)}, 200);
        addParallel(new Action[] {new ReachLineWaitAction(3.7 * Constants.DRIVE_ENCODER_TO_METERS), new ArmPoseAction(ArmPose.HYBRID), new RunIntakeAction(0.1)}, 1850);
        
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 3000);

    }
    
}
