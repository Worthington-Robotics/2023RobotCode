package frc.robot.actions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.SuperStructure;

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
			SmartDashboard.putNumber("SuperStructure/Action", 1.0);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			SuperStructure.getInstance().setIntakePower(0.0d);
			SmartDashboard.putNumber("SuperStructure/Action", 0);
			//TODO make sure this works and does not interfere with arm
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
