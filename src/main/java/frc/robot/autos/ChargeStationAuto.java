package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.BlockingMoveAction;
import frc.robot.actions.drive.BlockingSetDrivePowerAction;
import frc.robot.actions.drive.NonblockingMoveAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.vision.SnapshotAction;
import frc.robot.subsystems.Arm.ArmPose;

public class ChargeStationAuto extends StateMachineDescriptor{
    public ChargeStationAuto(){
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 200);
        addSequential(new PoseWaitAction(), 300);
        addSequential(new ArmPoseAction(ArmPose.MID), 200);
        addSequential(new PoseWaitAction(), 3000);
        addParallel(new Action[] {new BlockingMoveAction(15.5 * Constants.ENCODER_PER_INCH, 0), new AutoPipelineAction(0)}, 4000);
        //addParallel(new Action[] {new LLHoldAction(true, true), new SnapshotAction()}, 500); //use limelight for correction 9
        addParallel(new Action[] {new RunIntakeAction(Constants.ANYTHING_OUT_POWER), new SnapshotAction()}, 250);
        //addParallel(new Action[] {new UnblockingMoveAction(- 100 * Constants.ENCODER_PER_INCH, 0), new ReachLineWaitAction(false, - 12 * Constants.ENCODER_PER_INCH)}, 6000);
        //addParallel(new Action[] {new ArmPoseAction(ArmPose.TRANSIT), new ReachLineWaitAction(false, - 88 * Constants.ENCODER_PER_INCH)}, 200);
        addSequential(new NonblockingMoveAction(- 5 * Constants.ENCODER_PER_INCH, 0), 300);
        addSequential(new ReachLineWaitAction(false, - 5 * Constants.ENCODER_PER_INCH), 6000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        addSequential(new BlockingSetDrivePowerAction(0, -0.3), 2500);
        addSequential(new DriveLevelAction(0), 7000);



        // addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        // addSequential(new PoseWaitAction(), 300);
        // addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200);
        // addSequential(new PoseWaitAction(), 1750);
        // addParallel(new Action[] {new MoveForwardAction(15 * Constants.ENCODER_PER_INCH, 0), new AutoPipelineAction(LimelightPipeline.Low)}, 4000);
        // addSequential(new LLHoldAction(true, true), 3000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250); //release cone
        // addParallel(new Action[] {new UnblockingMoveAction(- 100 * Constants.ENCODER_PER_INCH, 0), new ReachLineWaitAction(false, - 12 * Constants.ENCODER_PER_INCH)}, 6000);
        // addParallel(new Action[] {new ArmPoseAction(ArmPose.TRANSIT), new ReachLineWaitAction(false, - 88 * Constants.ENCODER_PER_INCH)}, 200);
        // addSequential(new DriveLevelAction(0), 7000);

    }
 
}
