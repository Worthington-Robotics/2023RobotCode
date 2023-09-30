package frc.lib.statemachine;

import java.util.Collections;
import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class Action {
    private boolean hasStopped = false;

    /*
     * Empty constructor
     */
    public Action() {}

    /**
     * Code to run on action start
     */
    public abstract void onStart();

    /**
     * Code to run while action loops
     * <p>approx. every 20ms
     */
    public abstract void onLoop();

    /**
     * Method that tells the state machine the action is finished earlier than the scheduler
     *
     * @return true action is ready to self terminate
     */
    public abstract boolean isFinished();

    /**
     * Code to run when the action has been called by the state machine to stop
     */
    public abstract void onStop();

    /**
     * Non-implemented method by child class
     * <p>Prevents onStop from being called multiple times
     */
    public void doStop() {
        if (!hasStopped) {
            onStop();
            hasStopped = true;
        }
    }

    /**
     * Converts an action to a wpilib command for buttons
     */
    public static Command toCommand(Action action) {
        return new Command() {
            public void initialize() {
                action.onStart();
            }

            public Set<Subsystem> getRequirements() {
                return Collections.emptySet();
            }

            public void execute() {
                action.onLoop();
            }

            public boolean isFinished() {
                return action.isFinished();
            }

            public void end(boolean end) {
                action.onStop();
            }

        };
    }
}
