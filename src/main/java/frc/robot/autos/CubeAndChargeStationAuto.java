package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.LLHoldAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.BlockingMoveAction;
import frc.robot.actions.drive.NonblockingMoveAction;
import frc.robot.actions.drive.NonblockingSetDrivePowerAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.PositivePitchWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.vision.AutoPipelineAction;
import frc.robot.actions.vision.SnapshotAction;
import frc.robot.subsystems.Arm.ArmPose;

public class CubeAndChargeStationAuto extends StateMachineDescriptor{
    
    public CubeAndChargeStationAuto(){
        addSequential(new ArmPoseAction(ArmPose.FIRST_MOVE), 200);
        addSequential(new PoseWaitAction(), 300);
        addSequential(new ArmPoseAction(ArmPose.CONE_MID_FRONT), 200); //put the robot into the set pose
        addSequential(new PoseWaitAction(), 1750);
        addParallel(new Action[] {new RotateTurretAction(-190 * Constants.TURRET_TPD, false), new AutoPipelineAction(0)}, 0);
        addParallel(new Action[] {new LLHoldAction(true, true), new SnapshotAction()}, 3000); //use limelight for correction 9
        addParallel(new Action[] {new RunIntakeAction(Constants.ANYTHING_OUT_POWER), new SnapshotAction()}, 250);
        addSequential(new RotateTurretAction(0, false), 2000);
        addSequential(new ArmPoseAction(ArmPose.INTAKE_LITE), 0);
        addSequential(new PoseWaitAction(), 1750);
        addSequential(new TimeWaitAction(), 250);
        //END CONE
        addParallel(new Action[] {new NonblockingMoveAction(170 * Constants.ENCODER_PER_INCH, 0), new ReachLineWaitAction(false, 120 * Constants.ENCODER_PER_INCH)}, 8000);
        addParallel(new Action[] {new ArmPoseAction(ArmPose.INTAKE), new RunIntakeAction(Constants.INTAKE_POWER)}, 9000);
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 200);
        addParallel(new Action[] {new PoseWaitAction(), new DriveTurnAction(90)}, 2000);
        addSequential(new BlockingMoveAction(15 * Constants.ENCODER_PER_INCH, 0), 5000);
        addSequential(new DriveTurnAction(90), 1500);

        addSequential(new NonblockingSetDrivePowerAction(0, 0.5), 200);
        addSequential(new PositivePitchWaitAction(), 3000);

        addSequential(new DriveLevelAction(0), 7000);

    }
}
