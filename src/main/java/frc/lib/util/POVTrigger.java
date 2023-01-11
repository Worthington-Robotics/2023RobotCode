package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

public class POVTrigger extends Trigger
{
    Joystick joystick;
    int POVpos = 0;
    public POVTrigger(Joystick joystick)
    {
        this.joystick = joystick;
    }
    public POVTrigger(Joystick joystick, int POVpos)
    {
        this.joystick = joystick;
        this.POVpos = POVpos;
    }

    @Override
    public boolean get() {
        return joystick.getPOV() == POVpos;
    }
    /**
   * Starts the given command whenever the button is newly pressed.
   *
   * @param command the command to start
   */
  public void whenPressed(final Command command) {
    whenActive(command);
  }

  /**
   * Constantly starts the given command while the button is held.
   *
   * {@link Command#start()} will be called repeatedly while the button is held, and will be
   * canceled when the button is released.
   *
   * @param command the command to start
   */
  public void whileHeld(final Command command) {
    whileActive(command);
  }

  /**
   * Starts the command when the button is released.
   *
   * @param command the command to start
   */
  public void whenReleased(final Command command) {
    whenInactive(command);
  }

  /**
   * Toggles the command whenever the button is pressed (on then off then on).
   *
   * @param command the command to start
   */
  public void toggleWhenPressed(final Command command) {
    toggleWhenActive(command);
  }

  /**
   * Cancel the command when the button is pressed.
   *
   * @param command the command to start
   */
  public void cancelWhenPressed(final Command command) {
    cancelWhenActive(command);
  }

}