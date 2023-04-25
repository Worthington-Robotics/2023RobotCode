package frc.robot.autos;

import frc.lib.statemachine.Action;
import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.manipulator.AutoStartIntaking;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.actions.wait.PoseWaitAction;
import frc.robot.subsystems.Arm.ArmPose;

public class NoMoveAuto extends StateMachineDescriptor{

    public NoMoveAuto() {
        addSequential(new ZeroGyroAction(), 100);
        addParallel(new Action[] {new RunIntakeAction(0.1), new ArmPoseAction(ArmPose.HIGH)}, 250);
        addParallel(new Action[] {new RunIntakeAction(0.1), new PoseWaitAction()}, 1800);
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 1500);
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 200);
        addSequential(new PoseWaitAction(), 2500);

    }
    
}

