package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.wait.TimeWaitAction;

public class AutoOne extends StateMachineDescriptor {
    public AutoOne() {
        addSequential(new TimeWaitAction(), 2000);
        addSequential(new MoveForwardAction(320000, 0), 5000);
        // addSequential(new RunIntakeAction(0.5), 5000);
        addSequential(new TimeWaitAction(), 1000);
        //addSequential(new MoveForwardAction(15345, -90), 5000);
        addSequential(new MoveForwardAction(-320000, 0), 5000);
        addSequential(new TimeWaitAction(), 2000);
    }
}
