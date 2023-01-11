package frc.lib.loops;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope
 * calibration, etc.)
 */
public interface Loop {
    /**
     * what the loop runs when started by the subsystem manager
     *
     * @param timestamp handled by subsystem manager
     */
    public void onStart(double timestamp);

    /**
     * what the loop runs while run by the subsystem manager
     *
     * @param timestamp handled by subsystem manager
     */
    public void onLoop(double timestamp);

    /**
     * what the loop runs when ended by the subsystem manager
     *
     * @param timestamp handled by subsystem manager
     */
    public void onStop(double timestamp);
}
