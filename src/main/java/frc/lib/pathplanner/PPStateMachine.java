package frc.lib.pathplanner;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import frc.lib.statemachine.ActionGroup;
import frc.lib.util.Loggable;

public class PPStateMachine implements Loggable {
    private static final PPStateMachine instance = new PPStateMachine();
    private final LoggingData data = new LoggingData();

    public static PPStateMachine getInstance() {
        return instance;
    }

    private PPStateMachine() {}

    private final Runnable stateMachine = () -> {
        try {

        } catch (Exception e) {

        } finally {
            
        }
    };

    /**
     * Allows logging access to internal data structure of the state machine.
     * @return the state machines internal data class
     */
    public LoggingData getLogger() {
        return data;
    }

    public class LoggingData extends Loggable.LogData{
        public final AtomicInteger state = new AtomicInteger(-1);
        public final AtomicBoolean wantStop = new AtomicBoolean(true);
        public final AtomicBoolean stateLock = new AtomicBoolean(false);
        public volatile ActionGroup currentState;
        public volatile long t_start;
    }
}
