package frc.lib.statemachine;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachineDescriptor {
    private ConcurrentLinkedQueue<ActionGroup> queuedStates;

    public StateMachineDescriptor() {
        queuedStates = new ConcurrentLinkedQueue<>();
    }

    /**
     * Adds a sequential state to the state machine queue
     *
     * @param action     The action to be executed during the state
     * @param timeout_ms The timeout in ms of the state
     */
    public void addSequential(Action action, long timeout_ms) {
        queuedStates.add(new ActionGroup(action, timeout_ms));
    }

    /**
     * Adds a parallel state to the state machine queue
     *
     * @param actions    The array of actions to create a state from
     * @param timeout_ms The timeout in ms of the state
     */
    public void addParallel(Action[] actions, long timeout_ms) {
        queuedStates.add(new ActionGroup(actions, timeout_ms));
    }

    /**
     * Gets the queue object underlying the descriptor
     *
     * @return The queue containing all the states
     */
    public ConcurrentLinkedQueue<ActionGroup> getStates() {
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
