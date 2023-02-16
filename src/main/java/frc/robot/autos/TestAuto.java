package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;

public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
        // Shoot 3 while backing and tracking
        addSequential(new MoveForwardAction(170000, 0), 10000);
        addSequential(new DriveTurnAction(90), 10000);
        addSequential(new MoveForwardAction(70000, 90), 10000);
        addSequential(new DriveTurnAction(0), 10000);
        addSequential(new MoveForwardAction(90000, 0), 10000);
        addSequential(new DriveTurnAction(90), 10000);
        addSequential(new MoveForwardAction(220000, 90), 10000);
    }
}