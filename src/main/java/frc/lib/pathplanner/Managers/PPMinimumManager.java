package frc.lib.pathplanner.Managers;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.pathplanner.PPStateMachine;
import frc.lib.statemachine.Action;

public class PPMinimumManager extends Thread {
    private StopEvent event;
    private double waitTimeSec;
    private List<Thread> threadList = new ArrayList<>();
    private Thread deadlineThread;
    private Timer timer = new Timer();

    public PPMinimumManager(StopEvent event, double waitTimeSec) {
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
            while(!timer.hasElapsed(waitTimeSec)) {}
            for (Thread thread : threadList) {
                thread.interrupt();
            }
        } else if (event.executionBehavior == ExecutionBehavior.SEQUENTIAL) {
            while (!timer.hasElapsed(waitTimeSec)) {
                for (String name : event.names) {
                    Action action = PPStateMachine.getInstance().getActionFromName(name);
                    Thread thread = PPStateMachine.getInstance().newActionThread(action);
                    thread.start();
                    while (thread.isAlive() && !timer.hasElapsed(waitTimeSec)) {}
                }
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
            while(!timer.hasElapsed(waitTimeSec)) {}
            if (timer.hasElapsed(waitTimeSec)) {
                for (Thread startedThread : threadList) {
                    startedThread.interrupt();
                }
            }
        }
    }
}
