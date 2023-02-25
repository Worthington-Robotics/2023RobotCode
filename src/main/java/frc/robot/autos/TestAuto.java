package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.SetPositionAction;

public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
        addSequential(new SetPositionAction(129528, 75898, 0), 100);
        
        addSequential(new MoveForwardAction(170000, 0), 5000);
        addSequential(new DriveTurnAction(90), 5000);
        addSequential(new MoveForwardAction(70000, 90), 5000);
        addSequential(new DriveTurnAction(0), 5000);
        addSequential(new MoveForwardAction(90000, 0), 5000);
        addSequential(new DriveTurnAction(90), 5000);
        addSequential(new MoveForwardAction(220000, 90), 5000);

        addSequential(new DriveTurnAction(-90), 5000);
        // Reverse
        addSequential(new MoveForwardAction(220000, -90), 5000);
        addSequential(new DriveTurnAction(180), 5000);
        addSequential(new MoveForwardAction(90000, 180), 5000);
        addSequential(new DriveTurnAction(-90), 5000);
        addSequential(new MoveForwardAction(70000, -90), 5000);
        addSequential(new DriveTurnAction(180), 5000);
        addSequential(new MoveForwardAction(170000, 180), 5000);
        addSequential(new DriveTurnAction(0), 5000);
    }
}