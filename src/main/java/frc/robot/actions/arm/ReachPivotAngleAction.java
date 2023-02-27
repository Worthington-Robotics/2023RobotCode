package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ReachPivotAngleAction extends Action {
    double desiredDegree = 0.0;
    double startTime = Timer.getFPGATimestamp();
    // TODO: Add minimum times with error checking after merge

    public ReachPivotAngleAction (double theta) {
        this.desiredDegree = theta;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setDesiredPivot(desiredDegree);
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(Arm.getInstance().getPivotError()) < Constants.PIVOT_ANGLE_ACCEPTANCE
            && Timer.getFPGATimestamp() - startTime > Constants.PIVOT_MIN_TIME){
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void onStop() {
        // TODO Auto-generated method stub
        
    }
}