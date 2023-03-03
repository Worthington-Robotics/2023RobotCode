package frc.robot.actions.auto_poses;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class FormPoseAction extends Action{

    double[] inputVals = {0,0,0};
    double startTime = Timer.getFPGATimestamp();

    public FormPoseAction(double extensionEncoder, double pivotEncoder, double turretEncoder){
        inputVals[0] = extensionEncoder;
        inputVals[1] = pivotEncoder;
        inputVals[2] = turretEncoder;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setDesiredPivot(inputVals[0]);
        Arm.getInstance().setDesiredLength(inputVals[1]);
        Arm.getInstance().setDesiredTurret(inputVals[2]);
        Arm.getInstance().setClosedLoop();
        
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
