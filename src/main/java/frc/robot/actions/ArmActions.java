package frc.robot.actions;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.statemachine.Action;
import frc.robot.subsystems.Arm;

public class ArmActions {
	public static class SetTurretPowerAction extends Action {
		// The speed to run the arm at
		double power;
		
		public SetTurretPowerAction(double power) {
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
			Arm.getInstance().setTurretPower(0.0d);
		}

		@Override
		public boolean isFinished() {
			return false;
		} 
	}

	public static class SetExtensionPowerAction extends Action {
		// The speed to run the arm at
		double power;
		
		public SetExtensionPowerAction(double power) {
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
			Arm.getInstance().setExtensionPower(0.0d);
		}

		@Override
		public boolean isFinished() {
			return false;
		} 
	}

	public static class SetArmPowerAction extends Action {
		// The speed to run the arm at
		double power;
		
		public SetArmPowerAction(double power) {
			this.power = power;
		}

		@Override
		public void onStart() {
			Arm.getInstance().setArmPower(power);
		}

		@Override
		public void onLoop() {}

		@Override
		public void onStop() {
			Arm.getInstance().setArmPower(0.0d);
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
