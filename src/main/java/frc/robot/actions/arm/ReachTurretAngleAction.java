package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;


public class ReachTurretAngleAction extends Action {
    double desiredAngle = 0.0;
    double startTime = Timer.getFPGATimestamp();

    public ReachTurretAngleAction (double theta) {
        this.desiredAngle = theta;
    }

    @Override
    public void onStart() {
        Arm.getInstance().setDesiredTurret(desiredAngle);	
        Arm.getInstance().setClosedLoop();
    }

    @Override
    public void onLoop() { 
    }

    @Override
    public boolean isFinished() {
        // if (Math.abs(Arm.getInstance().getTurretError()) < Constants.TURRET_ANGLE_ENCODER_ACCEPTANCE
        //     && Timer.getFPGATimestamp() - startTime > Constants.TURRET_MIN_TIME){
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
