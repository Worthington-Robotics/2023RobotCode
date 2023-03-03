package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ReachPivotAngleAction extends Action {
    double desiredDegreeEncoder = 0.0;
    double startTime = Timer.getFPGATimestamp();

    public ReachPivotAngleAction (double theta) {
        this.desiredDegreeEncoder = theta;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setDesiredPivot(desiredDegreeEncoder);
        Arm.getInstance().setClosedLoop();
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // if (Math.abs(Arm.getInstance().getPivotError()) < Constants.PIVOT_ANGLE_ENCODER_ACCEPTANCE
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
