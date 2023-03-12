package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.SetPipelineAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.wait.TurretWaitAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class ChargeStationAuto extends StateMachineDescriptor{
    public ChargeStationAuto(){
       // addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200); // **going to be put on a different branch
        addSequential(new PoseWaitAction(), 300);
        //addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200);  // **going to be put on a different branch
        addSequential(new PoseWaitAction(), 1750);
        //addParallel(new Action[] {new MoveForwardAction(25 * Constants.ENCODER_PER_INCH, 0), new RotateTurretAction(-(15) * Constants.TURRET_TPD), new AutoPipelineAction(LimelightPipeline.Low)}, 6000); //move forward to get to the target
       // addSequential(new LLHoldAction(true, true), 3000); //use limelight for correction 9 // **going to be put on a different branch
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250); //release cone
        addSequential(new MoveForwardAction(- 80 * Constants.ENCODER_PER_INCH, 0), 6000);
        addSequential(new DriveLevelAction(0), 7000);
    }
 
}
