package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.AutoFieldRelativeAction;

public class TestAuto extends StateMachineDescriptor{

    public TestAuto(){
        addSequential(new AutoFieldRelativeAction(0.0, 1.0, 0.0), 2000);
    }
    
}
