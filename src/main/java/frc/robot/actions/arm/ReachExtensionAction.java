package frc.robot.actions.arm;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;
   
	public class ReachExtensionAction extends Action {
		double desiredLengthEncoder = 0.0;
		double startTime = Timer.getFPGATimestamp();

		public ReachExtensionAction (double length) {
			this.desiredLengthEncoder = length;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setDesiredLength(desiredLengthEncoder);	
			Arm.getInstance().setClosedLoop();
		}

		@Override
		public void onLoop() {
			
		}

		@Override
		public boolean isFinished() {
			// if (Math.abs(Arm.getInstance().getLengthError()) < Constants.EXTENSION_DISTANCE_ACCEPTED
			// 	&& Timer.getFPGATimestamp() - startTime > Constants.EXTENSION_MIN_TIME){
			// 	return true;
			// }
			// else {
			// 	return false;
			// }
			return true;
		}

		@Override
		public void onStop() {
			
		}

	}

	