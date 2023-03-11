package frc.robot.actions.wait;
<<<<<<< HEAD
import frc.robot.subsystems.Arm;
=======

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f
import frc.lib.statemachine.Action;
import frc.robot.Constants;


public class TurretWaitAction extends Action{

    @Override
    public void onStart() {
<<<<<<< HEAD

    }

    @Override
    public void onLoop() {        
=======
    }

    @Override
    public void onLoop() {
>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f
    }

    @Override
    public boolean isFinished() {
<<<<<<< HEAD
       if(Math.abs(Arm.getInstance().getTurretError()) < Constants.TURRET_ANGLE_ENCODER_ACCEPTANCE){
        return true;
       }
       return false;
    }

    @Override
    public void onStop() {   
=======
        if((Math.abs(Arm.getInstance().getTurretError()) < Constants.TURRET_ANGLE_ENCODER_ACCEPTANCE)){
            return true;
    }
    return false;
    }

    @Override
    public void onStop() {

>>>>>>> c2aec83e56e319ffe7203bd744b1e0a85f51523f
    }
    
}
