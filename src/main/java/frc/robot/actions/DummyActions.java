package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Dummy;
import frc.robot.Constants;

public class DummyActions {
	static public class DummyStop extends Action {
		@Override
		public void onStart() {
			if (Constants.DEBUG_LEVEL > 0) {
				System.out.println("Started first action");
			}
			Dummy.getInstance().setState(Dummy.State.kStopped);
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

	static public class DummyWait extends Action {
		@Override
		public void onStart() {
			if (Constants.DEBUG_LEVEL > 0) {
				System.out.println("Started second action");
			}
			Dummy.getInstance().setState(Dummy.State.kRunning);
		}
	
		@Override
		public void onLoop() {}
	
		@Override
		public void onStop() {}
	
		@Override
		public boolean isFinished() {
			return Dummy.getInstance().getCount() >= 5000;
		}
	}

	static public class DummyPrint extends Action {
		@Override
		public void onStart() {
			System.out.println(Dummy.getInstance().getCount());
		}
	
		@Override
		public void onLoop() {
			System.out.println("Running");
		}
	
		@Override
		public void onStop() {
			System.out.println("Done");
		}
	
		@Override
		public boolean isFinished() {
			return false;
		}
	}
}
