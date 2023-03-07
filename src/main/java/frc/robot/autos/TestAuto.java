package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.RotateTurretAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.SetPositionAction;
import frc.robot.actions.manipulator.RunIntakeAction;
import frc.robot.subsystems.Arm.ArmPose;

public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
        addSequential(new ArmPoseAction(ArmPose.UNSTOW), 3000);
        addSequential(new ArmPoseAction(ArmPose.CUBE_HIGH), 3000); //put the robot into the set pose
        addSequential(new RotateTurretAction(4370), 3413); //rotate turret 15 degrees left
        addSequential(new RunIntakeAction(Constants.ANYTHING_OUT_POWER), 500); //spit out the cube
        addSequential(new ArmPoseAction(ArmPose.TRANSIT), 3000);
        addSequential(new ArmPoseAction(ArmPose.STOWN), 10000);
    }
}