package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Stores a boolean that must remain true for a set duration before the data returns true.
 * (e.g. an object passing a sensor)
 *
 * @author Spazzinq (George Fang)
 */
public class TimerBoolean {
    private double waitDuration;
    private double startTimestamp;

    private boolean started;

    /**
     * Sets the wait duration and gets the current timestamp.
     * @param waitDuration the duration that must pass until the TimerBoolean returns true
     */
    public TimerBoolean(double waitDuration) {
        this.waitDuration = waitDuration;
    }

    /**
     * Returns true if the boolean is started and the current timestamp is past the wait duration.
     * @return if the given boolean has been true for a sufficient amount of time
     */
    public boolean getBoolean() {
        return started && startTimestamp != 0 && (startTimestamp + waitDuration < Timer.getFPGATimestamp());
    }

    /**
     * Sets the starting timestamp for the calculation.
     */
    public void start() {
        startTimestamp = Timer.getFPGATimestamp();
        started = true;
    }

    /**
     * Ends the timer, forcing the TimerBoolean to return false.
     */
    public void stop() {
        startTimestamp = 0;
        started = false;
    }

    /**
     * Resets the starting timestamp to the current time.
     */
    public void reset() {
        this.startTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Returns if the boolean is started.
     * @return if the boolean is started
     */
    public boolean isStarted() {
        return started;
    }

    /**
     * Sets the duration that must pass until the TimerBoolean returns true.
     * @param waitDuration the duration that must pass until the TimerBoolean returns true
     */
    public void setWaitDuration(double waitDuration) {
        this.waitDuration = waitDuration;
    }

    /**
     * Returns the duration that must pass until the TimerBoolean returns true.
     * @return the duration that must pass until the TimerBoolean returns true
     */
    public double getWaitDuration() {
        return waitDuration;
    }
}
