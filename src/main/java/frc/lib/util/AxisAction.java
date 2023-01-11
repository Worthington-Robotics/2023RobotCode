package frc.lib.util;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}.
 */
public class AxisAction extends AxisTrigger {
  private final GenericHID m_joystick;
  private final int m_axis;
  private final double m_trigger;
  private final boolean m_trigGreater;

  /**
   * Create a joystick button for triggering commands.
   *
   * @param joystick     The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
   * @param buttonNumber The button number (see {@link GenericHID#getRawButton(int) }
   */
  public AxisAction(GenericHID joystick, int axis, double trigger, boolean trigGreater) {
    m_joystick = joystick;
    m_axis = axis;
    m_trigger = trigger;
    m_trigGreater = trigGreater;
  }

  /**
   * Gets the value of the joystick button.
   *
   * @return The value of the joystick button
   */
  @Override
  public boolean get() {
    boolean get =  m_joystick.getRawAxis(m_axis) >= m_trigger;
    if(m_trigGreater){
    return get;
    }
    return !get;
  }
}
