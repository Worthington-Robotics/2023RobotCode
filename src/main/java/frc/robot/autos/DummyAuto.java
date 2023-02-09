package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.DummyActions;

public class DummyAuto extends StateMachineDescriptor {
    public DummyAuto() {
        // Shoot 3 while backing and tracking
        addSequential(new DummyActions.DummyWait(), 100000);
        addSequential(new DummyActions.DummyStop(), 100000);
    }
}