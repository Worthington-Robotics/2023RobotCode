package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;


public class TurretHoldAction extends Action {

    @Override
    public void onStart() {
        Arm.getInstance().turretHoldLock(true, Arm.getInstance().getTurretEncoder(), Arm.getInstance().getExtendEncoder());
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
        Arm.getInstance().turretHoldLock(false, 0, 0);
    }

} 
