package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.NonblockingSetDrivePowerAction;
import frc.robot.actions.drive.BlockingMoveAction;
import frc.robot.actions.drive.BlockingSetDrivePowerAction;
import frc.robot.actions.wait.LevelPitchWaitAction;
import frc.robot.actions.wait.NegativePitchWaitAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.PositivePitchWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.subsystems.Arm.ArmPose;

public class RedChargeStation extends StateMachineDescriptor{
    public RedChargeStation(){
        // addSequential(new ArmPoseAction(ArmPose.SLIDE), 200); //put the robot into the set pose
        // addSequential(new PoseWaitAction(), 1750);
        // addSequential(new RotateTurretAction(180 * Constants.TURRET_TPD, false), 4000);
        // addSequential(new ArmPoseAction(ArmPose.HIGH), 200); //put the robot into the set pose
        // addSequential(new AutoPipelineAction(1), 3000);
        // addSequential(new PoseWaitAction(), 7000);
        // addSequential(new LLHoldAction(true, true), 1000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200);   
        addSequential(new PoseWaitAction(), 1750);
        // addSequential(new RotateTurretAction(0  * Constants.TURRET_TPD, false), 4000);
        // addParallel(new Action[] {new RotateTurretAction(0 * Constants.TURRET_TPD, false)}, 4000);
        addSequential( new DriveTurnAction(-25), 4000);
        // addSequential(new BlockingSetDrivePowerAction(-25, 0.4), 2400);
        // addSequential(new DriveTurnAction(0), 2000);
        // addSequential(new DriveLevelAction(0), 7000);


        addSequential(new NonblockingSetDrivePowerAction(-25, 0.45), 1500);
        addSequential(new NegativePitchWaitAction(-11.0), 7000);

        //addSequential(new DriveTurnAction(0), 2000);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.45), 200);
        addSequential(new PositivePitchWaitAction(11.0), 7000);


        addSequential(new NonblockingSetDrivePowerAction(0, .45), 200);
        addSequential(new LevelPitchWaitAction(), 3000);
        addSequential(new BlockingMoveAction(30 * Constants.ENCODER_PER_INCH, 0), 3000);
        addSequential( new DriveTurnAction(-25), 4000);

        // addSequential(new TimeWaitAction(), 200);
        addSequential(new NonblockingSetDrivePowerAction(-25, -0.7), 200);
        addSequential(new PositivePitchWaitAction(13.0), 7000);
        // addSequential(new NonblockingSetDrivePowerAction(0, -.3), 200);
        // addSequential(new LevelPitchWaitAction(), 3000);

        addParallel(new Action[] {new DriveLevelAction(0), new TimeWaitAction()}, 9000);

    }
 
}