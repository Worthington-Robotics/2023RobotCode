package frc.lib.pathplanner;

import java.lang.reflect.Constructor;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.WaitBehavior;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.pathplanner.Managers.PPDeadlineManager;
import frc.lib.pathplanner.Managers.PPMinimumManager;
import frc.lib.pathplanner.Managers.PPNonSpecialManager;
import frc.lib.statemachine.Action;
import frc.lib.util.Loggable;
import frc.robot.subsystems.SwerveDrive;

public class PPStateMachine implements Loggable {
    //Creates a singleton of the state machine
    static PPStateMachine instance = new PPStateMachine();
    private LoggingData data = new LoggingData();
    public static PPStateMachine getInstance() {return instance;}

    private List<PathPlannerTrajectory> fullTrajectory = new ArrayList<>(); //The entire trajectory, a list of trajectories with path points
    private PathPlannerTrajectory currentTrajectory; //The current trajectory to be followed
    private List<Thread> threads = new ArrayList<>(); //A list of currently running threads, they are killed when disabled or the path is over
    private Thread mainThread; //This thread runs the state machine
    private Thread pathThread; //This thread is the one that runs the path planner action

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("Auto");
    private DoubleArrayPublisher fieldPub = table.getDoubleArrayTopic("Trajectory").publish();

    /**
     * The main Runnable of the state machine. This handles when and how to start and stop actions. As well as when and how to start and stop path planner following.
     */
    private Runnable stateMachine = () -> {
        try {
            data.state.set(0);
            while(data.state.get() < fullTrajectory.size()) {                                                       //While the trajectory is not over
                currentTrajectory = fullTrajectory.get(data.state.get());                                           //Get the current trajectory from the list
                StopEvent startStopEvent = currentTrajectory.getStartStopEvent();                                   //Get the current event
                pathThread = new PPPathStateMachine(currentTrajectory);                                             //Creates a new thread of the path state machine, the thing that actually follows the path
                pathThread.setName("Trajectory Action " + data.state.get() + " - " + currentTrajectory.getName());  //Simply setting the name of the thread
                if (DriverStation.getAlliance() == Alliance.Blue) {
                    logTrajectory(currentTrajectory);
                } else {
                    PathPlannerTrajectory transformedTraj = PathPlannerTrajectory.transformTrajectoryForAlliance(currentTrajectory, DriverStation.getAlliance());
                    logTrajectory(transformedTraj);
                }
                
                if(startStopEvent.waitBehavior == WaitBehavior.NONE) {
                    startAndWaitNonSpecial(startStopEvent);
                    pathThread.start();
                    System.out.println("[State Machine] None was selected");
                } else if(startStopEvent.waitBehavior == WaitBehavior.BEFORE) {
                    System.out.println("[State Machine] Before was selected");
                    Timer.delay(startStopEvent.waitTime);
                    startAndWaitNonSpecial(startStopEvent);
                    pathThread.start();
                } else if(startStopEvent.waitBehavior == WaitBehavior.AFTER) {
                    System.out.println("[State Machine] After was selected");
                    startAndWaitNonSpecial(startStopEvent);
                    Timer.delay(startStopEvent.waitTime);
                    pathThread.start();
                } else if(startStopEvent.waitBehavior == WaitBehavior.DEADLINE) {
                    System.out.println("[State Machine] Deadline was selected");
                    startAndWaitDeadline(startStopEvent, startStopEvent.waitTime);
                    pathThread.start();
                } else if(startStopEvent.waitBehavior == WaitBehavior.MINIMUM) {
                    System.out.println("[State Machine] Minimum was selected");
                    startAndWaitMinimum(startStopEvent, startStopEvent.waitTime);
                    pathThread.start();
                }
                pathThread.join();
                SwerveDrive.getInstance().setState(SwerveDrive.State.FieldRel);

                if (data.state.get() == fullTrajectory.size() - 1) { //At the very end
                    clearTrajectory();
                    StopEvent endStopEvent = currentTrajectory.getEndStopEvent();
                    if(endStopEvent.waitBehavior == WaitBehavior.NONE) {
                        startAndWaitNonSpecial(endStopEvent);
                    } else if(endStopEvent.waitBehavior == WaitBehavior.BEFORE) {
                        Timer.delay(endStopEvent.waitTime);
                        startAndWaitNonSpecial(endStopEvent);
                    } else if(endStopEvent.waitBehavior == WaitBehavior.AFTER) {
                        startAndWaitNonSpecial(endStopEvent);
                        Timer.delay(endStopEvent.waitTime);
                    } else if(endStopEvent.waitBehavior == WaitBehavior.DEADLINE) {
                        startAndWaitDeadline(endStopEvent, endStopEvent.waitTime);
                    } else if(endStopEvent.waitBehavior == WaitBehavior.MINIMUM) {
                        startAndWaitMinimum(endStopEvent, endStopEvent.waitTime);
                    }
                }
                System.out.println("[State Machine] Path " + data.state.get() + " Over");
                data.state.set(data.state.get() + 1);
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            currentTrajectory = fullTrajectory.get(0);
            assertStop();
        }
    };

    /**
     * Runs the state machine with the currently selected path
     */
    public void run() {
        mainThread = new Thread(stateMachine);
        mainThread.start();
    }

    /**
     * @param fullTrajectory The trajectory to set the state machine to use.
     */
    public void setPath(List<PathPlannerTrajectory> fullTrajectory) {
        this.fullTrajectory = fullTrajectory;
        this.currentTrajectory = fullTrajectory.get(0);
    }
    /**
     * Starts all of the events, following the Execution Behavior provided. This is for the wait behaviors of None, After, and Before.
     * @param event The event wanting to be followed.
     */
    private void startAndWaitNonSpecial(StopEvent event) {
        if (!event.names.isEmpty()) {
            PPNonSpecialManager manager = new PPNonSpecialManager(event);
            manager.start();
            try {
                manager.join();
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public int getState() {
        return data.state.get();
    }
    /**
     * Starts all of the events, following the Execution Behavior provided. This is for the wait behavior of Deadline.
     * @param event The event wanting to be followed.
     * @param waitTime The time maximum time that the actions can take.
     */
    private void startAndWaitDeadline(StopEvent event, double waitTime) {
        if (!event.names.isEmpty()) {
            PPDeadlineManager manager = new PPDeadlineManager(event, waitTime);
            manager.start();
            try {
                manager.join();
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    /**
     * Starts all of the events, following the Execution Behavior provided. This is for the wait behavior of Minimum.
     * @param event The event wanting to be followed.
     * @param waitTime The time minimum time that the actions can take.
     */
    private void startAndWaitMinimum(StopEvent event, double waitTime) {
        if (!event.names.isEmpty()) {
            PPMinimumManager manager = new PPMinimumManager(event, waitTime);
            manager.start();
            try {
                manager.join();
            } catch(InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    /**
     * Assures that all of the threads that were ever started are exited. This NEEDS to be called in disabledInit in Robot.java.
     */
    public void assertStop() {
        if(pathThread != null) {
            pathThread.interrupt();
        }
        if(mainThread != null) {
            mainThread.interrupt();
        }
        for (int i =0; i<threads.size(); i++) {
            threads.get(i).interrupt();
            threads.get(i).stop();
        }
        threads.clear();
        data.state.set(-1);
    }
    /**
     * Takes in an action and starts and manages a new thread that executes that action. This is needed due to sudden Disables while the state machine is running.
     * @param runAction The action that is turned into a thread.
     * @return The new thread that was just started.
     */
    public Thread newActionThread(Action runAction) {
        PPActionRunnable thread = new PPActionRunnable(runAction);
        thread.setDaemon(true);
        thread.setName(runAction.getClass().getName());
        thread.start();
        threads.add(thread);
        return thread;
    }
    /**
     * @deprecated
     * Use newActionThread to start a new action. This is needed for specific events.
     * @param thread The thread to add to the state machines thread management.
     */
    public void registerNewThread(Thread thread) {
        threads.add(thread);
    }

    public Action getActionFromName(String name) {
        try {
            Class<?> requestedClass = Class.forName("frc.robot.actions." + name);
            Constructor<?> constructor = requestedClass.getDeclaredConstructor();
            return (Action) constructor.newInstance();
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }
    /**
     * A wrapper to not directly interact with the Field2d class.
     * @param traj The trajectory to be logged.
     */
    private void logTrajectory(PathPlannerTrajectory traj) {
        List<Double> outList = new ArrayList<Double>();
        for(double time = 0; time < traj.getTotalTimeSeconds(); time+=0.1) {
            Pose2d currentPose = traj.sample(time).poseMeters;
            outList.add(currentPose.getX());
            outList.add(currentPose.getY());
            outList.add(currentPose.getRotation().getRadians());
        }
        fieldPub.set(outList.stream().mapToDouble(Double::doubleValue).toArray());
    }
    /**
     * Clears the tracjectory from the field element.
     */
    public void clearTrajectory() {
        fieldPub.set(new double[] {0.0, 0.0, 0.0});
    }

    @Override
    public LogData getLogger() {
        return data;
    }

    public class LoggingData extends Loggable.LogData{
        public final AtomicInteger state = new AtomicInteger(0);
    }
    
}
