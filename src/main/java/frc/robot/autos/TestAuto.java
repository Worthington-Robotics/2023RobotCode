package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.AutoFieldRelAction;

public class TestAuto extends StateMachineDescriptor{

    public TestAuto(){
        addSequential(new AutoFieldRelAction(0.0, 1.0, 0.0), 2000);
    }
    
}
