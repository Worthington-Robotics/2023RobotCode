package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.NonblockingSetDrivePowerAction;
import frc.robot.actions.drive.BlockingSetDrivePowerAction;
import frc.robot.actions.wait.NegativePitchWaitAction;
import frc.robot.actions.wait.PositivePitchWaitAction;
import frc.robot.actions.wait.LevelPitchWaitAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.vision.SnapshotAction;
import frc.robot.subsystems.Arm.ArmPose;

public class AdvancedChargeStation extends StateMachineDescriptor{
    public AdvancedChargeStation(){
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        // addSequential(new PoseWaitAction(), 3000);
        // addSequential(new ArmPoseAction(ArmPose.MID), 200);
        // addSequential(new PoseWaitAction(), 3000);

        // addSequential(new RotateTurretAction(-180 * Constants.TURRET_TPD, false), 2000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        // addSequential(new RotateTurretAction(0, false), 4000);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        // addSequential(new PoseWaitAction(), 3000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 200);
        addSequential(new PoseWaitAction(), 3000);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new NegativePitchWaitAction(), 3000);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new PositivePitchWaitAction(), 3000);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new LevelPitchWaitAction(), 3000);
        addSequential(new BlockingSetDrivePowerAction(0, .3), 500);

        addSequential(new TimeWaitAction(), 200);
        addSequential(new NonblockingSetDrivePowerAction(0, - 0.5), 200);
        addSequential(new PositivePitchWaitAction(), 3000);

        addSequential(new DriveLevelAction(0), 7000);

    }
 
}
