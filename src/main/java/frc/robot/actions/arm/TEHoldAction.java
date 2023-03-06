package frc.robot.actions.arm;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class TEHoldAction extends Action {

    @Override
    public void onStart() {
        Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder());
        Arm.getInstance().extendHoldLock(true, Arm.getInstance().getExtendEncoder());
    }

    @Override
    public void onLoop() { 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Arm.getInstance().turretHoldLock(false, 0);
        Arm.getInstance().extendHoldLock(false, 0);
    }

} 
