package frc.lib.statemachine;

import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;

public class ActionGroup {
    private LinkedList<Action> group;
    private double t_Start, t_Timeout;

    /**
     * Constructs an action group from an array of actions. Actions will 
     * be handled in the order they are added to the group
     * @param actions the array of actions to construc the group with
     * @param timeout_ms the timeout of the whoe group in msec
     */
    ActionGroup(Action[] actions, long timeout_ms) {
        t_Timeout = (double) timeout_ms / 1000.000;
        group = new LinkedList<>();
        for (Action action : actions) {
            group.add(action);
        }

    }

    /**
     * Constructs an action group frm a single action. Actions will 
     * be handled in the order they are added to the group
     * @param action the single action to contruct the group from
     * @param timeout_ms the timeout of the group in msec
     */
    ActionGroup(Action action, long timeout_ms) {
        t_Timeout = (double) timeout_ms / 1000.000;
        group = new LinkedList<>();
        group.add(action);
    }

    /**
     * Runs the on-start code for each action in the group.
     * Also records the start time of state
     */
    public void onStart() {
        t_Start = Timer.getFPGATimestamp();
        group.forEach(Action::onStart);
    }

    /**
     * Runs the on-loop code for each action in the group.
     */
    public void onLoop() {
        group.forEach(Action::onLoop);
    }

    /**
     * Determines if the state is ready to be advanced. This can
     * happen forcefully on a timeout or if all action in the 
     * group time out individually. 
     * <p>This also handles calling the stop code on actions who
     * have completed independently.
     *
     * @return true if all actions in the group have finished or
     * if they should be forcibly timed out
     */
    public boolean isFinished() {
        if (t_Start + t_Timeout <= Timer.getFPGATimestamp()) {
            return true;
        }
        boolean temp = true;
        for (Action action : group) {
            if (action.isFinished()) {
                action.doStop();
            } else {
                temp = false;
            }
        }
        return temp;
    }

    /**
     * forcibly terminate all actions within the group
     */
    public void onStop() {
        group.forEach(Action::doStop);
    }

    /**
     * gets a string representation of the class names inside the action group
     * @return a string of all class names in the group
     */
    public String toString(){
        String classNames = "";
        for (int i = 0; i < group.size(); i++) {
            classNames += group.get(i).getClass().getSimpleName();
            if(i < group.size() - 1) classNames += " ";
        }
        return classNames;
    }

}

