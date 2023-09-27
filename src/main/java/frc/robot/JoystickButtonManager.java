package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.arm.ArmHighPose;
import frc.robot.actions.arm.ArmMidPose;
import frc.robot.actions.arm.ArmPoseAction;
import frc.robot.actions.arm.ArmSlidePose;
import frc.robot.actions.arm.ArmTestKinematics;
import frc.robot.actions.arm.ArmUnstowPose;
import frc.robot.actions.drive.ToggleGranny;
import frc.robot.actions.drive.ZeroGyro;
import frc.robot.actions.lights.SetCone;
import frc.robot.actions.lights.SetCube;
import frc.robot.actions.lights.TestBackCurrent;
import frc.robot.actions.manipulator.IntakeGamePiece;
import frc.robot.actions.manipulator.ManualInAction;
import frc.robot.actions.manipulator.ManualOutAction;
import frc.robot.actions.manipulator.MoveWristAction;
import frc.robot.actions.manipulator.SpitGamePiece;
import frc.robot.subsystems.arm.Arm.ArmPose;

public class JoystickButtonManager {
    public JoystickButtonManager() {
        registerButtons();
    }

    public void registerButtons() {
        new JoystickButton(Constants.Joysticks.XBOX, 1).whileTrue(Action.toCommand(new SetCone()));
        new JoystickButton(Constants.Joysticks.XBOX, 2).whileTrue(Action.toCommand(new SetCube()));
        new JoystickButton(Constants.Joysticks.XBOX, 3).whileTrue(Action.toCommand(new TestBackCurrent()));
        new JoystickButton(Constants.Joysticks.XBOX, 5).whileTrue(Action.toCommand(new ToggleGranny()));
        new JoystickButton(Constants.Joysticks.XBOX, 6).whileTrue(Action.toCommand(new ZeroGyro()));
        // new JoystickButton(Constants.Joysticks.XBOX, 3).whileTrue(Action.toCommand(new ArmTestKinematics()));

        new JoystickButton(Constants.Joysticks.SECOND, 1).whileTrue(Action.toCommand(new ArmUnstowPose()));
        new JoystickButton(Constants.Joysticks.SECOND, 2).whileTrue(Action.toCommand(new ArmSlidePose()));
        new JoystickButton(Constants.Joysticks.SECOND, 3).whileTrue(Action.toCommand(new ManualOutAction()));
        new JoystickButton(Constants.Joysticks.SECOND, 4).whileTrue(Action.toCommand(new ManualInAction()));
        new JoystickButton(Constants.Joysticks.SECOND, 5).whileTrue(Action.toCommand(new MoveWristAction(.33)));
        new JoystickButton(Constants.Joysticks.SECOND, 6).whileTrue(Action.toCommand(new MoveWristAction(-.33)));
        new JoystickButton(Constants.Joysticks.SECOND, 7).whileTrue(Action.toCommand(new ArmMidPose()));
        new JoystickButton(Constants.Joysticks.SECOND, 8).whileTrue(Action.toCommand(new ArmHighPose()));

        new JoystickButton(Constants.Joysticks.SECOND, 11).whileTrue(Action.toCommand(new ArmPoseAction(ArmPose.HYBRID)));
        new JoystickButton(Constants.Joysticks.SECOND, 12).whileTrue(Action.toCommand(new ArmPoseAction(ArmPose.INTAKE)));
    }
    
}
