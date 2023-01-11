package frc.lib.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {
    /**
     * What the loop runs when started by the subsystem manager
     *
     * @param timestamp Handled by subsystem manager
     */
    public void onStart(double timestamp);

    /**
     * What the loop runs while run by the subsystem manager
     *
     * @param timestamp Handled by subsystem manager
     */
    public void onLoop(double timestamp);

    /**
     * What the loop runs when ended by the subsystem manager
     *
     * @param timestamp Handled by subsystem manager
     */
    public void onStop(double timestamp);
}
