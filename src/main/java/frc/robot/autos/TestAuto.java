package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.DriveTurn;
import frc.robot.actions.drive.MoveForward;

public class TestAuto extends StateMachineDescriptor {
    public TestAuto() {
        // Shoot 3 while backing and tracking
        // addSequential(new DummyActions.DummyWait(), 100000);
        addSequential(new MoveForward(150000, 0), 10000);
        addSequential(new DriveTurn(90), 10000);
    }
}