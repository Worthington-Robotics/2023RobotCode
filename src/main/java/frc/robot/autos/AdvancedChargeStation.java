package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.NonblockingSetDrivePowerAction;
import frc.robot.actions.drive.BlockingMoveAction;
import frc.robot.actions.wait.NegativePitchWaitAction;
import frc.robot.actions.wait.PositivePitchWaitAction;
import frc.robot.actions.wait.LevelPitchWaitAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.subsystems.Arm.ArmPose;

public class AdvancedChargeStation extends StateMachineDescriptor{
    public AdvancedChargeStation(){
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200);
        addSequential(new PoseWaitAction(), 5000);
        addSequential(new RotateTurretAction(180 * Constants.TURRET_TPD, false), 4000);
        //addParallel(new Action[] {new PoseWaitAction(), new RotateTurretAction(180 * Constants.TURRET_TPD, false)}, 5000);
        addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        addSequential(new PoseWaitAction(), 5000);

        addSequential(new BlockingMoveAction(-15 * Constants.ENCODER_PER_INCH, 0), 5000);


        addSequential(new LLHoldAction(true, true), 1000);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new BlockingMoveAction(25 * Constants.ENCODER_PER_INCH, 0), 5000);
        addSequential(new ArmPoseAction(ArmPose.SLIDE), 200);
    
        addParallel(new Action[] {new PoseWaitAction(), new RotateTurretAction(0 * Constants.TURRET_TPD, false)}, 5000);
        addSequential(new TimeWaitAction(), 250);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new NegativePitchWaitAction(), 7000);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new PositivePitchWaitAction(), 7000);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new LevelPitchWaitAction(), 7000);

        addSequential(new TimeWaitAction(), 200);
        addSequential(new NonblockingSetDrivePowerAction(0, -0.5), 200);
        addSequential(new PositivePitchWaitAction(), 7000);
        addSequential(new TimeWaitAction(), 2000);

        addParallel(new Action[] {new DriveLevelAction(0), new TimeWaitAction()}, 9000);

    }
 
}
