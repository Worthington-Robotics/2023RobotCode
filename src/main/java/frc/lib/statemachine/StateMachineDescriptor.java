package frc.lib.statemachine;

import java.util.concurrent.ConcurrentLinkedQueue;

public class StateMachineDescriptor {
    private ConcurrentLinkedQueue<ActionGroup> queuedStates;

    public StateMachineDescriptor() {
        queuedStates = new ConcurrentLinkedQueue<>();
    }

    /**
     * adds a sequential state to the state machine queue
     *
     * @param action     the action to be executed during the state
     * @param timeout_ms the timeout in ms of the state
     */
    public void addSequential(Action action, long timeout_ms) {
        queuedStates.add(new ActionGroup(action, timeout_ms));
    }

    /**
     * adds a parallel state to the state machine queue
     *
     * @param actions    the array of actions to create a state from
     * @param timeout_ms the timeout in ms of the state
     */
    public void addParallel(Action[] actions, long timeout_ms) {
        queuedStates.add(new ActionGroup(actions, timeout_ms));
    }

    /**
     * gets the queue object underlying the descriptor
     *
     * @return the queue containing all the states
     */
    public ConcurrentLinkedQueue<ActionGroup> getStates() {
        return queuedStates;
    }

    /**
     * code to run when the state machine is first started
     * <p>does not have to be an action. this can be any real code
     */
    public void onStart() {
    }

    /**
     * code to run when the state machine is stopped
     * <p>does not have to be an action. this can be any real code
     */
    public void onStop() {
    }
}
