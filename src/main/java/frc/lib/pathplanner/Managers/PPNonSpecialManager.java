package frc.lib.pathplanner.Managers;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;

import frc.lib.pathplanner.PPStateMachine;
import frc.lib.statemachine.Action;

public class PPNonSpecialManager extends Thread {
    private StopEvent event;
    private List<Thread> threadList = new ArrayList<>();
    private Thread deadlineThread;

    public PPNonSpecialManager(StopEvent event) {
        this.event = event;
    }

    @Override
    public void run() {
        if(event.executionBehavior == ExecutionBehavior.PARALLEL) {
            for(String name : event.names) { //Start the actions
                Action action = PPStateMachine.getInstance().getActionFromName(name);
                Thread thread = PPStateMachine.getInstance().newActionThread(action);
                thread.start();
                threadList.add(thread);
            }
            while(!areCommandsFinishedParallel()) {} //Acts as a wait
        } else if (event.executionBehavior == ExecutionBehavior.SEQUENTIAL) {
            for(String name : event.names) {
                Action action = PPStateMachine.getInstance().getActionFromName(name);
                Thread thread = PPStateMachine.getInstance().newActionThread(action);
                thread.start();
                try {
                    thread.join();
                } catch (InterruptedException e) {
                    e.printStackTrace();
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
            while(deadlineThread.isAlive()) {}
            if (!deadlineThread.isAlive()) {
                for (Thread startedThread : threadList) {
                    startedThread.interrupt();
                }
            }
        }
    }

    private boolean areCommandsFinishedParallel() { //Returns true if all actions are done
        boolean flag = true;
        for (Thread thread : threadList) {
            flag = flag && !thread.isAlive();
        }
        return flag;
    }
}
