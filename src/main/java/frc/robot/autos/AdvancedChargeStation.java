package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.BlockingSetDrivePowerAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.subsystems.Arm.ArmPose;

public class AdvancedChargeStation extends StateMachineDescriptor{
    public AdvancedChargeStation(){
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new RotateTurretAction(180 * Constants.TURRET_TPD, false), 4000);
        addSequential(new ArmPoseAction(ArmPose.MID), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200); //put the robot into the set pose
        addSequential(new AutoPipelineAction(1), 3000);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new LLHoldAction(true, true), 1000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.MID), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200);
        addSequential(new PoseWaitAction(), 1750);
    
        addSequential(new RotateTurretAction(0 * Constants.TURRET_TPD, false), 4000);
        addSequential(new BlockingSetDrivePowerAction(0, 0.3), 2800);
        addSequential(new DriveLevelAction(0), 7000);
        // addParallel(new Action[] {new PoseWaitAction(), new NonblockingSetDrivePowerAction(0, 0.66)}, 1200);
        // addSequential(new NegativePitchWaitAction(), 7000);

        // addSequential(new NonblockingSetDrivePowerAction(0, 0.55), 200);
        // addSequential(new PositivePitchWaitAction(), 7000);

        // addSequential(new NonblockingSetDrivePowerAction(0, 0.66), 200);
        // addSequential(new LevelPitchWaitAction(), 7000);

        // addSequential(new TimeWaitAction(), 200);
        // addSequential(new NonblockingSetDrivePowerAction(0, -0.66), 200);
        // addSequential(new PositivePitchWaitAction(), 7000);
        // addSequential(new TimeWaitAction(), 1000);

        // addParallel(new Action[] {new DriveLevelAction(0), new TimeWaitAction()}, 9000);

    }
 
}
