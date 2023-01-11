package frc.lib.util;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class DebouncedJoystickButton extends DebouncedButton {
  private final GenericHID m_joystick;
  private final int m_buttonNumber;

  /**
   * Create a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public DebouncedJoystickButton(GenericHID joystick, int buttonNumber) {
    m_joystick = joystick;
    m_buttonNumber = buttonNumber;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    return m_joystick.getRawButton(m_buttonNumber);
  }
}
