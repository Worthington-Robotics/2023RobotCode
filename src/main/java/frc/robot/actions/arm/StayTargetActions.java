package frc.robot.actions.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class StayTargetActions {
	
	public static class StayTurretPowerAction extends Action {
		// The speed to run the turret motor at
		double power;
		
		public StayTurretPowerAction(double power) {
			this.power = power;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setTurretPower(power);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			//Arm.getInstance().setTurretPower(0.0d);
		}

		@Override
		public boolean isFinished() {
			return false;
		} 
	}

	public static class StayExtensionPowerAction extends Action {
		// The speed to run the extension motor at
		double power;
		
		public StayExtensionPowerAction(double power) {
			this.power = power;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setExtensionPower(power);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			//Arm.getInstance().setExtensionPower(0.0d);
		}

		@Override
		public boolean isFinished() {
			return false;
		} 
	}

	public static class StayPivotPowerAction extends Action {
		// The speed to run the arm at
		double power;
		
		public StayPivotPowerAction(double power) {
			this.power = power;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setPivotPower(power);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			//Arm.getInstance().setPivotPower(0.0d);
		}

		@Override
		public boolean isFinished() {
			return false;
		} 
	}

	public static class GrabAction extends Action {
		@Override
		public void onStart() {
			Arm.getInstance().setGrabber(DoubleSolenoid.Value.kForward);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			Arm.getInstance().setGrabber(DoubleSolenoid.Value.kReverse);
		}

		@Override
		public boolean isFinished() {
			return false;
		} 
	}
}
