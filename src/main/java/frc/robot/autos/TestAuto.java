package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.AutoFieldRelAction;
import frc.robot.actions.drive.AutoRobotRelAction;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.drive.DriveNonblockingTurnAction;
import frc.robot.actions.wait.ReachLineWaitAction;

public class TestAuto extends StateMachineDescriptor{

    public TestAuto(){
       // addSequential(new AutoFieldRelAction(0.0, 1.0, 0.0), 2000);
        addSequential(new DriveNonblockingLineAction(4, 4, (-1.5 * Constants.DRIVE_ENCODER_TO_METERS), 0, 1), 5000);
        //addSequential(new ReachLineWaitAction(-1.5 * Constants.DRIVE_ENCODER_TO_METERS), 5000);
        //addSequential(new DriveNonblockingTurnAction(90), 2000);
        //addSequential(new AutoRobotRelAction(-2, 0, 0), 3000);
    }
    
}
