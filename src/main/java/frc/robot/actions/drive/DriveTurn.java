package frc.robot.actions.drive;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveTurn extends Action {
    double heading;
    public DriveTurn(double heading) {
        this.heading = heading;
    }

    @Override
    public void onStart() {
        DriveTrain.getInstance().setDesiredHeading(heading);
        DriveTrain.getInstance().setAnglePID();
    }

    @Override
    public void onLoop() {
        // TODO Auto-generated method stub
        
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if(Math.abs(DriveTrain.getInstance().getHeadingError()) < Constants.ANGLE_ACCEPTANCE){ //checks exit conditions
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void onStop() {
        DriveTrain.getInstance().resetEncoders();
        // TODO Auto-generated method stub
        
    }
}
