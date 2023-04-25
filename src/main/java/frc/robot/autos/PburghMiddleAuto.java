package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.AutoFieldRelAction;
import frc.robot.actions.drive.AutoLevelAction;
import frc.robot.actions.drive.AutoRobotRelAction;
import frc.robot.actions.drive.DriveFlipGyroZero;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.manipulator.AutoSetIntakeTime;
import frc.robot.actions.manipulator.AutoStartIntaking;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.actions.wait.ReachLineWaitAction;
import frc.robot.subsystems.Arm.ArmPose;
import frc.lib.statemachine.Action;

public class PburghMiddleAuto extends StateMachineDescriptor{

    public PburghMiddleAuto() {
        addSequential(new ZeroGyroAction(), 100);
        addParallel(new Action[] {new RunIntakeAction(0.1), new ArmPoseAction(ArmPose.HIGH)}, 250);
        addParallel(new Action[] {new RunIntakeAction(0.1), new PoseWaitAction()}, 1800);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);

        // addSequential(new AutoStartIntaking(), 100);
        // addSequential(new ZeroGyroAction(), 100);
        // addSequential(new ArmPoseAction(ArmPose.HIGH), 200);
        // addSequential(new PoseWaitAction(), 5000);
        // addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 250);
        // addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        
        addSequential(new PoseWaitAction(), 2500);


        addSequential(new AutoFieldRelAction(-3, 0, 0), 1000);
        addSequential(new AutoLevelAction(), 7000);
        addSequential(new DriveFlipGyroZero(), 200);

    }
    
}
