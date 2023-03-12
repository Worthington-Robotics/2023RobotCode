package frc.robot.actions.wait;
import frc.robot.subsystems.Arm;
import frc.lib.statemachine.Action;
import frc.robot.Constants;


public class TurretWaitAction extends Action{

    @Override
    public void onStart() {

    }

    @Override
    public void onLoop() {        
    }

    @Override
    public boolean isFinished() {
       if(Math.abs(Arm.getInstance().getTurretError()) < Constants.TURRET_ANGLE_ENCODER_ACCEPTANCE){
        return true;
       }
       return false;
    }

    @Override
    public void onStop() {   
    }
    
}
