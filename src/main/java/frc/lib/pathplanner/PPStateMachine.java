package frc.lib.pathplanner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.lib.util.Loggable;

public class PPStateMachine implements Loggable {
    private static final PPStateMachine instance = new PPStateMachine();
    private final LoggingData data = new LoggingData();
    private volatile PPStateMachineDescriptor descriptor;
    private Notifier thread;

    private final Map<Action, Boolean> currentActions = new HashMap<>();
    private final List<PathPlannerTrajectory.EventMarker> unpassedMarkers = new ArrayList<>();
    private final Timer timer = new Timer();
    private boolean isFinished = true;

    public static PPStateMachine getInstance() {
        return instance;
    }

    private PPStateMachine() {}

    private final Runnable stateMachine = () -> {
        try {
            System.out.println("Path Planner State Machine Starting Execution");

        } catch (Exception e) {

        } finally {
            // Run all cleanup procedures
            SmartDashboard.putNumber("StateMachine/state", data.state.get());
            data.stateLock.set(false);
            data.wantStop.set(true);
        }
    };

    public boolean runMachine(PPStateMachineDescriptor descrip) {
        if(data.stateLock.get()) return false;

        data.stateLock.set(true);
        data.wantStop.set(false);

        descriptor = descrip;

        isFinished = false;

        currentActions.clear();
    
        unpassedMarkers.clear();
        unpassedMarkers.addAll(descriptor.getPathMarkers());
    
        timer.reset();
        timer.start();
    
        // pathFollowingCommand.initialize();
        // currentActions.put(pathFollowingCommand, true);

        thread = new Notifier(stateMachine);
        thread.setName("Path Planner State Machine");
        thread.startSingle(0);

        return true;
    }

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
        public final AtomicBoolean isFinished = new AtomicBoolean(true);
        public volatile PathPlannerTrajectory.EventMarker currentState;
        public volatile long t_start;
    }
}
