package frc.robot.actions.manipulator;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ReachWristAngleAction extends Action{
    private double desiredDegree = 0.0;
    private double startTime = Timer.getFPGATimestamp();

    public ReachWristAngleAction (double theta) {
        this.desiredDegree = theta;
    }

    @Override
    public void onStart() {
        Manipulator.getInstance().setDesiredWristAngle(desiredDegree);
        Manipulator.getInstance().setClosedLoop();  
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return (Math.abs(Manipulator.getInstance().getWristEncoderError()) < Constants.WRIST_ANGLE_ENCODER_ACCEPTANCE
            && Timer.getFPGATimestamp() - startTime > Constants.WRIST_MIN_TIME);
    }

    @Override
    public void onStop() {}
}
