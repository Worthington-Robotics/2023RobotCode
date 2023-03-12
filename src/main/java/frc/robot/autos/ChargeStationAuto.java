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
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.wait.TurretWaitAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.subsystems.VisionLink.LimelightPipeline;

public class ChargeStationAuto extends StateMachineDescriptor{
    public ChargeStationAuto(){
        addSequential(new MoveForwardAction(-25 * Constants.ENCODER_PER_INCH, 0), 6000); //move forward to get to the target
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 2000);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 2000);
        addSequential(new TimeWaitAction(), 500);

        addSequential(new MoveForwardAction(25 * Constants.ENCODER_PER_INCH, 0), 6000); //move forward to get to the target
        addSequential(new RotateTurretAction(-(15) * Constants.TURRET_TPD), 200); //rotate turret 15 degrees right 6
        addSequential(new TurretWaitAction(), 2000);
        addSequential(new AutoPipelineAction(LimelightPipeline.Low), 200);
        addSequential(new LLHoldAction(true, true), 4000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500);

        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new PoseWaitAction(), 7000);
        addSequential(new ArmPoseAction(ArmPose.STOWN), 3000);
        addSequential(new PoseWaitAction(), 7000);

        addSequential(new MoveForwardAction(- 50 * Constants.ENCODER_PER_INCH, 0), 6000);
        addSequential(new DriveLevelAction(0), 7000);
    }
 
}
