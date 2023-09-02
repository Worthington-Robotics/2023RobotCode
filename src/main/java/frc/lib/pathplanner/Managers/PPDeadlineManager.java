package frc.lib.pathplanner.Managers;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.pathplanner.PPStateMachine;
import frc.lib.statemachine.Action;

public class PPDeadlineManager extends Thread {
    private StopEvent event;
    private double waitTimeSec;
    private List<Thread> threadList = new ArrayList<>();
    private Thread deadlineThread;
    private Timer timer = new Timer();

    public PPDeadlineManager(StopEvent event, double waitTimeSec) {
        this.event = event;
        this.waitTimeSec = waitTimeSec;
    }

    @Override
    public void run() {
        timer.reset();
        timer.start();
        if(event.executionBehavior == ExecutionBehavior.PARALLEL) {
            for (String name : event.names) {
                Action action = PPStateMachine.getInstance().getActionFromName(name);
                Thread thread = PPStateMachine.getInstance().newActionThread(action);
                thread.start();
                threadList.add(thread);
            }
            while(!areCommandsFinishedParallel() || !timer.hasElapsed(waitTimeSec)) {}
            for (Thread thread : threadList) {
                thread.interrupt();
            }
        } else if (event.executionBehavior == ExecutionBehavior.SEQUENTIAL) {
            Thread subManagerThread = new Thread(subManager);
            subManagerThread.setName("Deadline Sequential Manager Thread");
            subManagerThread.start();
            while (subManagerThread.isAlive() || !timer.hasElapsed(waitTimeSec)) {}
            subManagerThread.interrupt();
            for(Thread thread : threadList) {
                thread.interrupt();
            }
        } else if (event.executionBehavior == ExecutionBehavior.PARALLEL_DEADLINE) {
            Action deadlineAction = PPStateMachine.getInstance().getActionFromName(event.names.get(0));
            deadlineThread = PPStateMachine.getInstance().newActionThread(deadlineAction);
            deadlineThread.start();
            for (int i =0; i<event.names.size() -1; i++) {
                Action action = PPStateMachine.getInstance().getActionFromName(event.names.get(i + 1));
                Thread thread = PPStateMachine.getInstance().newActionThread(action);
                thread.start();
                threadList.add(thread);
            }
            while(deadlineThread.isAlive() && !timer.hasElapsed(waitTimeSec)) {}
            if (!deadlineThread.isAlive() || timer.hasElapsed(waitTimeSec)) {
                for (Thread startedThread : threadList) {
                    startedThread.interrupt();
                }
            }
        }
    }

    private Runnable subManager = () -> {
        for (String name : event.names) {
            Action action = PPStateMachine.getInstance().getActionFromName(name);
            Thread thread = PPStateMachine.getInstance().newActionThread(action);
            thread.start();
            while (thread.isAlive() && !timer.hasElapsed(waitTimeSec)) {}
        }
    };

    private boolean areCommandsFinishedParallel() { //Returns true if all actions are done
        boolean flag = true;
        for (Thread thread : threadList) {
            flag = flag && !thread.isAlive();
        }
        return flag;
    }
}
