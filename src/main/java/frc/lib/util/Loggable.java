package frc.lib.util;

/**
 * A base level class for inhertiance to allow for the creation 
 * of a list of loggable classes.
 */
public interface Loggable {
    /**
     * A base level data class for use caputring logging data.
     * 
     * This inheritance can be used to combine lower level logging in subsystems
     * as well as inside major components
     */
    public static class LogData {}

    public abstract LogData getLogger();
}
