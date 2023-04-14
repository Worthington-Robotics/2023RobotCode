package frc.robot.actions.wait;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.statemachine.Action;

public class ReachLineWaitAction extends Action{

    double line;

    public ReachLineWaitAction(double line) {
        this.line = line;
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onLoop() {
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(DriveTrain.getInstance().getAverageEncoder()) >= Math.abs(line));
    }

    @Override
    public void onStop() {
    }

    
}
