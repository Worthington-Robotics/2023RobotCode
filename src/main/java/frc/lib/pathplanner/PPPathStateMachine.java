package frc.lib.pathplanner;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.actions.PPSwerveControllerAction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class PPPathStateMachine extends Thread {
    private PathPlannerTrajectory trajectory;
    private Thread pathThread;
    private List<EventMarker> markers = new ArrayList<>();
    private List<Action> startedActions = new ArrayList<>();

    public PPPathStateMachine(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
        this.markers = trajectory.getMarkers();
    }

    @Override
    public void run() { //oh my god this is terrible
        PPSwerveControllerAction pathAction = new PPSwerveControllerAction(trajectory, SwerveDrive.getInstance().getPoseSupplier(), Constants.DriveTrain.DRIVE_X_CONTROLLER, Constants.DriveTrain.DRIVE_Y_CONTROLLER, Constants.DriveTrain.DRIVE_ROTATION_CONTROLLER, SwerveDrive.getInstance().setModuleStates(), true);
        pathThread = new PPActionRunnable(pathAction);
        pathThread.setName("Path Action - " + trajectory.getName());
        PPStateMachine.getInstance().registerNewThread(pathThread);
        markers = trajectory.getMarkers();
        pathThread.start();

        while(pathThread.isAlive()) {
            for(EventMarker marker : markers) {
                if (SwerveDrive.getInstance().getState() == SwerveDrive.State.PathPlanner && !pathThread.isAlive()) {
                    break;
                }
                if (pathAction.getCurrentTime() >= marker.timeSeconds) {
                    for(String name : marker.names){
                        if(startedActions.size() == 0) {
                            Action action = PPStateMachine.getInstance().getActionFromName(name);
                            PPStateMachine.getInstance().newActionThread(action);
                            startedActions.add(action);
                        } 
                        else {
                            for (Action startedAction : startedActions) {
                                if(!startedAction.getClass().getCanonicalName().equalsIgnoreCase(PPStateMachine.getInstance().getActionFromName(name).getClass().getCanonicalName())) {
                                    Action action = PPStateMachine.getInstance().getActionFromName(name);
                                    startedActions.add(action);
                                    PPStateMachine.getInstance().newActionThread(action);
                                }
                            }
                        }
                    }
                }
            }
        } 
    }
}
