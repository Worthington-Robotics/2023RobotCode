package frc.robot.autos;

import frc.lib.statemachine.StateMachineDescriptor;
import frc.robot.actions.drive.DriveTurnAction;
import frc.robot.actions.drive.MoveForwardAction;
import frc.robot.actions.wait.WaitAction;

public class AutoTwoFromRookie extends StateMachineDescriptor {
    public AutoTwoFromRookie() {
        addSequential(new WaitAction(), 1000);//place cone
        addSequential(new MoveForwardAction(81020, 0), 4000);   //move forwards until barrier
        addSequential(new DriveTurnAction(90), 3000);                                  //turn left to face cone
        addSequential(new MoveForwardAction(339000, 90), 6000); //move forwards to pickup cone
        addSequential(new WaitAction(), 1000); //wait for cone
        addSequential(new MoveForwardAction(-339000, 90), 6000); //move backwards to community
        addSequential(new DriveTurnAction(-180), 3000); // turn perpendicular to scoring pads
        addSequential(new MoveForwardAction(81020, -180), 5000); // move in front of scoring pad
        addSequential(new WaitAction(), 1000); // Place cone
        addSequential(new MoveForwardAction(-81020, -180), 5000); // move backwards until barrier
        addSequential(new DriveTurnAction(90), 3000); // Turn outwards
        addSequential(new MoveForwardAction(211875, 90), 6000); // move forward 125 in
        addSequential(new DriveTurnAction(122.6192431), 2000); // turn 32.62 degrees to face cone
        addSequential(new MoveForwardAction(115060, 135), 5000); // move to cone
    }
}