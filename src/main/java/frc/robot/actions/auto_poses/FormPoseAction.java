package frc.robot.actions.auto_poses;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class FormPoseAction extends Action{

    double buttonNumber;

    public FormPoseAction(double number){
        buttonNumber = number;
    }

    @Override
    public void onStart() {
       if (buttonNumber == 1) {
            Arm.getInstance().setPoseOne();
       } else if (buttonNumber == 2){
            Arm.getInstance().setPoseTwo();
        } else{
            Arm.getInstance().setPoseNothing();
        }
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
        
    }

    
}
