package frc.lib.statemachine;

import edu.wpi.first.wpilibj.command.Command;

public abstract class Action {

    private boolean hasStopped = false;

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

            protected void initialize() {
                action.onStart();
            }

            protected void execute() {
                action.onLoop();
            }

            protected boolean isFinished() {
                return action.isFinished();
            }

            protected void end() {
                action.onStop();
            }
        };
    }
}
