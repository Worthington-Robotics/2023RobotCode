package frc.robot.actions.arm;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class AllowExtensionPowerAction extends Action{

    @Override
    public void onStart() {
        Arm.getInstance().setExtensionButtonPressedTrue();  
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
        Arm.getInstance().setExtensionButtonPressedFalse();
        
    }

}