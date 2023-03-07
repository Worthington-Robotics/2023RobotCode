package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.DriveLevelAction;
import frc.robot.actions.drive.MoveForwardAction;

public class ChargeStationAuto extends StateMachineDescriptor{
    public ChargeStationAuto(){ 
        addSequential(new MoveForwardAction(100000, 0), 4000);
        addSequential(new DriveLevelAction(0), 7000);
    }
}
