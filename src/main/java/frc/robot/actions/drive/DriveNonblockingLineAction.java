package frc.robot.actions.drive;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.control.RotationalTrapController;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.DriveTrain;

public class DriveNonblockingLineAction extends Action {
    public double xMax;
    public double yMax;
    public double xDelta;
    public double yDelta;
    public double thetaAbs;

    public DriveNonblockingLineAction(double xMax, double yMax, double xDelta, double yDelta, double thetaAbs) {
        this.xMax = xMax;
        this.yMax = yMax;
        this.xDelta = xDelta;
        this.yDelta = yDelta;
        this.thetaAbs = thetaAbs;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setXMax(xMax);
        DriveTrain.getInstance().setYMax(yMax);
        DriveTrain.getInstance().setXDelta(xDelta);
        DriveTrain.getInstance().setYDelta(yDelta);
        DriveTrain.getInstance().setEndDesiredEncoder(xDelta);
        DriveTrain.getInstance().setThetaAbs(thetaAbs); 
        DriveTrain.getInstance().setAutoState();
    }

    @Override
    public void onLoop() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void onStop() {
        
    }
    
}
