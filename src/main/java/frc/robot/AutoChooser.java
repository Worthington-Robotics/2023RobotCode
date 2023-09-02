package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.pathplanner.PPStateMachine;

// Class that picks an autonomous routine to run based on user input
public class AutoChooser {
	private static AutoChooser instance = new AutoChooser();
	public static AutoChooser getInstance() { return instance; }
	private List<List<PathPlannerTrajectory>> autoList = new ArrayList<>(); //This is not a mistake
	private List<PathPlannerTrajectory> chosen;

	public void run() {
		PPStateMachine.getInstance().setPath(chosen);
		PPStateMachine.getInstance().run();
	}
	/**
	 * Checks the deploy directory and registers all of the detected autos locally.
	 */
	public void getAutos() {
		//Get the pathplanner directory
		File pathPlannerDir = new File(Filesystem.getDeployDirectory().getAbsolutePath() + "/pathplanner");
		File[] files = pathPlannerDir.listFiles();

		//Add each path to the list of autos
		for (File file : files) {
			if (file.getName().endsWith(".path")) {
				String name = file.getName().substring(0, file.getName().length() - 5);
				List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup(name, Constants.DriveTrain.PATH_CONSTRAINTS);
				autoList.add(traj);
			}
		}
		//set the chosen auto as the first auto
		chosen = autoList.get(0);
	}
	/**
	 * @param traj The trajectory to be changed to.
	 */
	public void change(List<PathPlannerTrajectory> traj) {
		chosen = traj;
	}
	/**
	 * @param name The trajectory's name to be changed to.
	 */
	public void change(String name) {
		for(int i =0; i<autoList.size();i++) {
			if(autoList.get(i).get(0).getName().equalsIgnoreCase(name)) {
				chosen = autoList.get(i);
			}
		}
	}
	/**
	 * Checks SmartDashboard to get the currently selected auto, then starts the state machine with that auto.
	 */
	public void runFromSelection() {
		final String string = SmartDashboard.getString("Auto Selector", chosen.get(0).getName());
		change(string);
		run();
	}
	/**
	 * Prints all of the registered autos to SmartDashboard.
	 */
	public void printAutos() {
		String[] listOfAutos = new String[autoList.size()];
		for (int i = 0; i<autoList.size();i++) {
			listOfAutos[i] = autoList.get(i).get(0).getName();
		}
		System.out.println(listOfAutos);
		SmartDashboard.putStringArray("Auto List", listOfAutos);
	}
	/**
	 * Logs the currently chosen auto to SmartDashboard.
	 */
	public void logAuto() {
		SmartDashboard.putString("Auto/Current", chosen.get(0).getName());
	}
}
