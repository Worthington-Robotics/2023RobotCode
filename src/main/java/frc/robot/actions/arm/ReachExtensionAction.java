package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
   
	public class ReachExtensionAction extends Action {
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

	