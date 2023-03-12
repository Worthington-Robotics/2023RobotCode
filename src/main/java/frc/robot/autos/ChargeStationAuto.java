package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.SetPipelineAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.wait.TurretWaitAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class ChargeStationAuto extends StateMachineDescriptor{
    public ChargeStationAuto(){
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 300);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new MoveForwardAction(10 * Constants.ENCODER_PER_INCH, 0), 5000);
        
        addSequential(new LLHoldAction(true, true), 3000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250); //release cone
        addSequential(new MoveForwardAction(- 50 * Constants.ENCODER_PER_INCH, 0), 6000);
        addSequential(new DriveLevelAction(0), 7000);
    }
 
}
