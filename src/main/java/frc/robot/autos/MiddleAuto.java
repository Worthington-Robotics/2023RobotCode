package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.AutoFieldRelAction;
import frc.robot.actions.drive.AutoLevelAction;
import frc.robot.actions.drive.AutoRobotRelAction;
import frc.robot.actions.drive.DriveFlipGyroZero;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.drive.AutoTurnAction;
import frc.robot.actions.drive.NonblockingDrivePowerAction;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.manipulator.AutoStartIntaking;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.LevelPitchWaitAction;
import frc.robot.actions.wait.NegativePitchWaitAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.PositivePitchWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.actions.wait.TimeWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class MiddleAuto extends StateMachineDescriptor{

    public MiddleAuto() {
        addSequential(new ZeroGyroAction(), 100);
        addParallel(new Action[] {new RunIntakeAction(0.1), new ArmPoseAction(ArmPose.HIGH)}, 250);
        addParallel(new Action[] {new RunIntakeAction(0.1), new PoseWaitAction()}, 1800);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);


        // Using the gyro and pitch //
        // addParallel(new Action[] {new PoseWaitAction(), new NonblockingDrivePowerAction(-3, 0, 0)}, 1200);
        // addSequential(new NegativePitchWaitAction(-11), 7000);

        // addSequential(new NonblockingDrivePowerAction(-2, 0, 0), 200);
        // addSequential(new PositivePitchWaitAction(11), 7000);

        // addSequential(new NonblockingDrivePowerAction(-2, 0, 0), 200);
        // addSequential(new LevelPitchWaitAction(3), 3000);

        // addSequential(new TimeWaitAction(), 200);
        // addSequential(new NonblockingDrivePowerAction(2, 0, 0), 200);
        // addSequential(new PositivePitchWaitAction(13), 7000);
        // addSequential(new TimeWaitAction(),1000);

        // addParallel(new Action[] {new AutoLevelAction(), new TimeWaitAction()}, 9000);


        ///////////////////////////////
        // using time //
        // addParallel(new Action[] {new PoseWaitAction(), new AutoFieldRelAction(-3, 0, 0)}, 1200);
        // addSequential(new AutoFieldRelAction(-2.5, 0, 0), 3000);
        // addSequential(new AutoFieldRelAction(2.5, 0, 0), 2800);
        // addSequential(new AutoLevelAction(), 9000);





    }
    
}
