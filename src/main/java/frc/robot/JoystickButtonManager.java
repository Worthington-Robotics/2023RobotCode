package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.statemachine.Action;
import frc.robot.actions.debug.logAction;

public class JoystickButtonManager {
    public JoystickButtonManager() {
        registerButtons();
    }

    public void registerButtons() {
        new JoystickButton(Constants.Joysticks.XBOX, 1).whileTrue(Action.toCommand(new logAction()));
    }
    
}
