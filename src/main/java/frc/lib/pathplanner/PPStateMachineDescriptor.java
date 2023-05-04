package frc.lib.pathplanner;

import java.util.concurrent.ConcurrentLinkedQueue;

import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;

import frc.lib.statemachine.Action;

public class PPStateMachineDescriptor {
    private ConcurrentLinkedQueue<PPActionGroup> queuedStates;

    public PPStateMachineDescriptor() {
        queuedStates = new ConcurrentLinkedQueue<>();
    }

    /**
     * Adds a sequential state to the state machine queue
     *
     * @param action     The action to be executed during the state
     * @param timeout_ms The timeout in ms of the state
     */
    public void addSequential(Action action, String marker, long timeout_ms) {
        queuedStates.add(new PPActionGroup(action, marker, timeout_ms));
    }

    /**
     * Adds a parallel state to the state machine queue
     *
     * @param actions    The array of actions to create a state from
     * @param timeout_ms The timeout in ms of the state
     */
    public void addParallel(Action[] actions, String marker, long timeout_ms) {
        queuedStates.add(new PPActionGroup(actions, marker, timeout_ms));
    }

    /**
     * Gets the queue object underlying the descriptor
     *
     * @return The queue containing all the states
     */
    public ConcurrentLinkedQueue<PPActionGroup> getStates() {
        return queuedStates;
    }

    /**
     * Code to run when the state machine is first started
     * <p>Does not have to be an action. This can be any real code
     */
    public void onStart() {
    }

    /**
     * Code to run when the state machine is stopped
     * <p>Does not have to be an action. This can be any real code
     */
    public void onStop() {
    }
}
