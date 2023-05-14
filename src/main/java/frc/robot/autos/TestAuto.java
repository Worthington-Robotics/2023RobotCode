package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.Constants;
import frc.robot.actions.drive.DriveNonblockingLineAction;
import frc.robot.actions.drive.ZeroGyroAction;
import frc.robot.actions.drive.AutoTurnAction;
import frc.robot.actions.wait.ReachLineWaitAction;

public class TestAuto extends StateMachineDescriptor{

    public TestAuto(){
       addSequential(new ZeroGyroAction(), 250);
       addSequential(new DriveNonblockingLineAction(-5.0, 0, -2 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
       addSequential(new ReachLineWaitAction(-2 * Constants.DRIVE_ENCODER_TO_METERS), 6000);
       addSequential(new DriveNonblockingLineAction(-5.0, 0, 2 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
       addSequential(new ReachLineWaitAction(2 * Constants.DRIVE_ENCODER_TO_METERS), 6000);
       addSequential(new DriveNonblockingLineAction(-5.0, 0, -2 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
       addSequential(new ReachLineWaitAction(-2 * Constants.DRIVE_ENCODER_TO_METERS), 6000);
       addSequential(new DriveNonblockingLineAction(-5.0, 0, 2 * Constants.DRIVE_ENCODER_TO_METERS, 0, 0), 5000);
       addSequential(new ReachLineWaitAction(2 * Constants.DRIVE_ENCODER_TO_METERS), 6000);
       addSequential(new AutoTurnAction(Math.PI), 6000);
    }
    
}
