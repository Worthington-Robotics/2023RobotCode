package frc.lib.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class provides an easy way to link commands to OI inputs.
 *
 * <p>
 * It is very easy to link a button to a command. For instance, you could link
 * the trigger button of a joystick to a "score" command.
 *
 * <p>
 * This class represents a subclass of Trigger that is specifically aimed at
 * buttons on an operator interface as a common use case of the more generalized
 * Trigger objects. This is a simple wrapper around Trigger with the method
 * names renamed to fit the Button object use.
 */
public abstract class DebouncedButton extends Trigger {
    /**
     * Starts the given command whenever the button is newly pressed.
     *
     * @param command The command to start
     */
    public void whenPressed(final Command command) {
        whenActive(command);
    }

    /**
     * Constantly starts the given command while the button is held.
     *
     * {@link Command#start()} will be called repeatedly while the button is held,
     * and will be canceled when the button is released.
     *
     * @param command The command to start
     */
    public void whileHeld(final Command command) {
        whileActiveContinuous(command);
    }

    /**
     * Starts the command when the button is released.
     *
     * @param command The command to start
     */
    public void whenReleased(final Command command) {
        whenInactive(command);
    }

    /**
     * Toggles the command whenever the button is pressed (on then off then on).
     *
     * @param command The command to start
     */
    public void toggleWhenPressed(final Command command) {
        TimerBoolean debouncer = new TimerBoolean(.1);
        while (!debouncer.getBoolean()) {
            if (!debouncer.isStarted())
            debouncer.start();
        }
        toggleWhenActive(command);
    }

    /**
     * Cancel the command when the button is pressed.
     *
     * @param command The command to start
     */
    public void cancelWhenPressed(final Command command) {
        cancelWhenActive(command);
    }
}
