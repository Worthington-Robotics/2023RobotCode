package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.RunIntakeAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.WaitAction;
import frc.robot.subsystems.DriveTrain;

public class AutoTwo extends StateMachineDescriptor{
    public AutoTwo(){
        addSequential(new WaitAction(), 2000);
        addSequential(new DriveTurnAction(-90), 5000);
        addSequential(new MoveForwardAction(76725, -90), 5000);
    }
    

}
