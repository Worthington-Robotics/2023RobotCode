package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ReturnActions {
    public static class ReturnPivotToBottomAction extends Action {
		double desiredDegree = 0.0;
		double startTime = Timer.getFPGATimestamp();
		// TODO: Add minimum times with error checking after merge

		public ReturnPivotToBottomAction (double theta) {
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

	public static class RetractExtensionAction extends Action {
		double desiredLength = 0.0;
		double startTime = Timer.getFPGATimestamp();

		public RetractExtensionAction (double length) {
			this.desiredLength = length;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setDesiredLength(desiredLength);	
		}

		@Override
		public void onLoop() {
			
			
		}

		@Override
		public boolean isFinished() {
			if (Math.abs(Arm.getInstance().getLengthError()) < Constants.EXTENSION_DISTANCE_ACCEPTED
				&& Timer.getFPGATimestamp() - startTime > Constants.EXTENSION_MIN_TIME){
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

	public static class ReturnTurretAction extends Action {
		double desiredAngle = 0.0;
		double startTime = Timer.getFPGATimestamp();

		public ReturnTurretAction (double theta) {
			this.desiredAngle = theta;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setDesiredTurretDegree(desiredAngle);	
		}

		@Override
		public void onLoop() {
			// TODO Auto-generated method stub
			
		}

		@Override
		public boolean isFinished() {
			if (Math.abs(Arm.getInstance().getTurretError()) < Constants.TURRET_ANGLE_ACCEPTANCE
				&& Timer.getFPGATimestamp() - startTime > Constants.TURRET_MIN_TIME){
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
}
