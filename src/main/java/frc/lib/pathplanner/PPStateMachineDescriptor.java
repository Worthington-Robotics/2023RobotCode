package frc.lib.pathplanner;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathPlannerTrajectory;
import frc.lib.statemachine.Action;

public class PPStateMachineDescriptor {
    private PathPlannerTrajectory path = new PathPlannerTrajectory();
    private List<PathPlannerTrajectory.EventMarker> pathMarkers = path.getMarkers();
    private Map<String, Action> eventMap = new HashMap<>();
    
    /**
     * Adds a sequential state to the state machine queue
     *
     * @param markerName     The marker at which the action will be executed.
     * @param actionName     The action to be executed.
     */
    public void addSequential(String markerName, Action actionName) {
        eventMap.put(markerName, actionName);
    }

    /**
     * Adds a parallel state to the state machine
     *
     * @param markerName     The marker at which the action will be executed.
     * @param actionArray     The array of actions that will be executed at the same marker.
     */
    public void addParallel(String markerName, Action[] actionArray) {
        for(Action action : actionArray) {
            eventMap.put(markerName, action);
        }
    }
     /**
     * Sets the state machine's path
     *
     * @param traj     The trajectory to be set.
     */
    public void setPath(PathPlannerTrajectory traj) {
        this.path = traj;
        this.pathMarkers = traj.getMarkers();
    }

    /*
     * Returns the current state of state machine!
     */
    public List<PathPlannerTrajectory.EventMarker> getPathMarkers() {
        return pathMarkers;
    }
}
