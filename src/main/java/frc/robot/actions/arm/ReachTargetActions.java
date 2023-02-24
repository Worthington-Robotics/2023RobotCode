package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ReachTargetActions {
    public static class ReachPivotAngleAction extends Action {
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

	public static class ReachExtensionAction extends Action {
		double desiredLength = 0.0;
		double startTime = Timer.getFPGATimestamp();

		public ReachExtensionAction (double length) {
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

	public static class ReachTurretAction extends Action {
		double desiredAngle = 0.0;
		double startTime = Timer.getFPGATimestamp();

		public ReachTurretAction (double theta) {
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
