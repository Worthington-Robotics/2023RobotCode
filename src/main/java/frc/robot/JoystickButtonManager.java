package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.arm.FollowTrajectory;
import frc.robot.actions.lights.SetCone;
import frc.robot.actions.lights.SetCube;

public class JoystickButtonManager {
    public JoystickButtonManager() {
        registerButtons();
    }

    public void registerButtons() {
        new JoystickButton(Constants.Joysticks.XBOX, 1).whileTrue(Action.toCommand(new SetCone()));
        new JoystickButton(Constants.Joysticks.XBOX, 2).whileTrue(Action.toCommand(new SetCube()));
        new JoystickButton(Constants.Joysticks.SECOND, 1).whenPressed(Action.toCommand(new FollowTrajectory()));
    }
    
}
