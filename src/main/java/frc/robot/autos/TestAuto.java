package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.AutoFieldRelAction;
import frc.robot.actions.drive.AutoRobotRelAction;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.drive.AutoTurnAction;
import frc.robot.actions.wait.ReachLineWaitAction;

public class TestAuto extends StateMachineDescriptor{

    public TestAuto(){
       addSequential(new ZeroGyroAction(), 250);
       addSequential(new AutoTurnAction(Math.PI), 3000);
      // addSequential(new DriveNonblockingLineAction(1, 0,  5 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
    }
    
}
