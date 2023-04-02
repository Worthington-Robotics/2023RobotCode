package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.Constants;
import frc.robot.subsystems.Arm.ArmPose;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.wait.PoseWaitAction;

public class PitTestAuto extends StateMachineDescriptor {
    // TODO: Adjust these values so its not 10 seconds between each pose
    int patience = 200;
    int pose_patience = 5000;
    int intake_patience = 5000;
    int drive_patience = 2000;

    public PitTestAuto() {
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new RotateTurretAction(180 * Constants.TURRET_TPD, false), 4000);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200); //put the robot into the set pose
        addSequential(new AutoPipelineAction(1), 3000);
        addSequential(new PoseWaitAction(), 7000);
        // addSequential(new RotateTurretAction(180 * Constants.TURRET_TPD, false), pose_patience);
        // addSequential(new TimeWaitAction(), drive_patience);
        // addSequential(new RotateTurretAction(0, false), pose_patience);
        // addSequential(new TimeWaitAction(), drive_patience);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), patience);
        // addSequential(new PoseWaitAction(), pose_patience);
        // addSequential(new TimeWaitAction(), drive_patience);
        // addSequential(new ArmPoseAction(ArmPose.MID), patience);
        // addSequential(new PoseWaitAction(), pose_patience);
        // addSequential(new TimeWaitAction(), drive_patience);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), patience);
        // addSequential(new PoseWaitAction(), pose_patience);
        // addSequential(new TimeWaitAction(), drive_patience);
        // addSequential(new ArmPoseAction(ArmPose.ZERO), patience);

    }
}
