package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.SetDrivePowerAction;
import frc.robot.actions.drive.UnblockingMoveAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class ChargeStationAuto extends StateMachineDescriptor{
    public ChargeStationAuto(){
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 300);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200);
        addSequential(new PoseWaitAction(), 3000);
        addParallel(new Action[] {new MoveForwardAction(15.5 * Constants.ENCODER_PER_INCH, 0), new AutoPipelineAction(LimelightPipeline.Low)}, 4000);
        addSequential(new LLHoldAction(true, true), 3000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250); //release cone
        //addParallel(new Action[] {new UnblockingMoveAction(- 100 * Constants.ENCODER_PER_INCH, 0), new ReachLineWaitAction(false, - 12 * Constants.ENCODER_PER_INCH)}, 6000);
        //addParallel(new Action[] {new ArmPoseAction(ArmPose.TRANSIT), new ReachLineWaitAction(false, - 88 * Constants.ENCODER_PER_INCH)}, 200);
        addSequential(new UnblockingMoveAction(- 5 * Constants.ENCODER_PER_INCH, 0), 300);
        addSequential(new ReachLineWaitAction(false, - 5 * Constants.ENCODER_PER_INCH), 6000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        addSequential(new SetDrivePowerAction(0, -0.3), 3000);
        addSequential(new DriveLevelAction(0, false), 7000);



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
