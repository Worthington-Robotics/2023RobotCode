package frc.lib.util;

    

/**
 * A base level class for inhertiance to allow for the creation 
 * of a list of loggable classes.
 */
public interface Logable{
    /**
     * A base level data class for use caputring logging data
     * this inheritance can be used to combine lower level logging in subsystems
     * as well as inside major components
     */
    public static class LogData{

    }

    public abstract LogData getLogger();
}

