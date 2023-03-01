package frc.robot.actions.auto_poses;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class FormPoseAction extends Action{

    double[] inputVals = {0,0,0};
    double startTime = Timer.getFPGATimestamp();

    public FormPoseAction(double pivotEncoder, double extensionEncoder, double turretEncoder){
        inputVals[0] = pivotEncoder;
        inputVals[1] = extensionEncoder;
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
    //     if (Math.abs(Arm.getInstance().getPivotError()) < Constants.PIVOT_ANGLE_ENCODER_ACCEPTANCE
    //     && Math.abs(Arm.getInstance().getLengthError()) < Constants.EXTENSION_DISTANCE_ACCEPTED
    //     && Math.abs(Arm.getInstance().getTurretError()) < Constants.TURRET_ANGLE_ENCODER_ACCEPTANCE
    //     && Timer.getFPGATimestamp() - startTime > Constants.PIVOT_MIN_TIME){
    //     return true;
    // }
    // else {
    //     return false;
    // }
        return true;
    }

    @Override
    public void onStop() {
        
    }

    
}
