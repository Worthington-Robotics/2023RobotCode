package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.IntakePosition;

public class SuperstructureActions {
	public static class RunIntakeUntilFinishedAction extends Action {
		// The speed to run the intake at
		double power;
		
		public RunIntakeUntilFinishedAction(double power) {
			this.power = power;
		}

		@Override
		public void onStart() {
			SuperStructure.getInstance().setIntakePower(power);
			//TODO make sure this works and does not interfere with arm
			
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			SuperStructure.getInstance().setIntakePower(0.0d);
			//TODO make sure this works and does not interfere with arm
			
		}

		@Override
		public boolean isFinished() {
			return SuperStructure.getInstance().isFinished();
		} 
	}
	
	public static class RunIntakeAction extends Action {
		// The speed to run the intake at
		double power;
		
		public RunIntakeAction(double power) {
			this.power = power;
		}

		@Override
		public void onStart() {
			SuperStructure.getInstance().setIntakePower(power);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			SuperStructure.getInstance().setIntakePower(0.0d);
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public static class MoveIntakeAction extends Action {
		SuperStructure.IntakePosition position;
		
		public MoveIntakeAction(SuperStructure.IntakePosition position) {
			this.position = position;
		}

		@Override
		public void onStart() {
			SuperStructure.getInstance().setIntakePosition(position);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

}
