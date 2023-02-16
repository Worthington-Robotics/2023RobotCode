package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.RunIntakeAction;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.drive.WaitAction;
import frc.robot.subsystems.DriveTrain;

public class AutoOne extends StateMachineDescriptor{
    public AutoOne(){
        addSequential(new WaitAction(), 2000);
        addSequential(new DriveTurnAction(-90), 5000);
        addSequential(new MoveForwardAction(320000, -90), 5000);
        // addSequential(new RunIntakeAction(0.5), 5000);
        addSequential(new WaitAction(), 5000);
        //addSequential(new MoveForwardAction(15345, -90), 5000);
        addSequential(new DriveTurnAction(90), 5000);
        addSequential(new MoveForwardAction(320000, 90), 5000);
        addSequential(new DriveTurnAction(180), 5000);
        addSequential(new WaitAction(), 2000);
    }
    

}
