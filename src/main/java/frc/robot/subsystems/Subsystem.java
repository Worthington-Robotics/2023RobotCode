package frc.robot.subsystems;

import frc.lib.loops.ILooper;
import frc.lib.util.Loggable;

//design pattern for caching periodic writes to avoid hammering the HAL/CAN.
public abstract class Subsystem implements Loggable {

    /**
     * Updates all periodic variables and sensors
     */
    public abstract void readPeriodicInputs();

    /**
     * Required for the subsystem's looper to be registered to the state machine
     * not required for subsystems that do not use looper
     *
     * @param enabledLooper the subsystem's Looper
     */
    public void registerEnabledLoops(ILooper enabledLooper) {}

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    public abstract void writePeriodicOutputs();

    /**
     * Outputs all logging information to the SmartDashboard
     */
    public abstract void outputTelemetry();

    /**
     * Called to reset and configure the subsystem
     */
    public abstract void reset();

    /**
     * Called to stop the autonomous functions of the subsystem and place it in open loop
     */
    public void onStop() {}

    public LogData getLogger() {
        return null;
    }

    /**
     * Inheritable data class for capuring log data from the subsystems
     */
    public class PeriodicIO extends Loggable.LogData {}
}