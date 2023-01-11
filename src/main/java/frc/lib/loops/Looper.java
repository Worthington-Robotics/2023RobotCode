package frc.lib.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * This code runs all of the robot's loops. Loop objects are stored in a List object. They are started when the robot
 * powers up and stopped after the match.
 */
public class Looper implements ILooper {
    public final double kPeriod = Constants.LOOPER_DT;

    private boolean running_;

    private final Notifier notifier_;
    private final List<Loop> loops_ = new ArrayList<>();
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;
    private double dt_ = 0;

    public Looper() {
        notifier_ = new Notifier(runnable_);
        running_ = false;
    }

    private final Runnable runnable_ = () -> {
        try{
            synchronized (taskRunningLock_) {
                if (running_) {
                    double now = Timer.getFPGATimestamp();
    
                    for (Loop loop : loops_) {
                        loop.onLoop(now);
                    }
    
                    dt_ = now - timestamp_;
                    timestamp_ = now;
                }
            }
        } catch(Exception e){
            System.out.println("Crash occured on iteration of loop");
            handleCrash(e);
        }
    };

    @Override
    public synchronized void register(Loop loop) {
        synchronized (taskRunningLock_) {
            loops_.add(loop);
        }
    }

    public synchronized void start() {
        if (!running_) {
            System.out.println("Starting loops");
            try{
                synchronized (taskRunningLock_) {
                    timestamp_ = Timer.getFPGATimestamp();
                    for (Loop loop : loops_) {
                        loop.onStart(timestamp_);
                    }
                    running_ = true;
                }
                notifier_.startPeriodic(kPeriod);
            }  catch(Exception e){
                System.out.println("Crash occured on startup of loop");
                handleCrash(e);
            } 
        }
    }

    public synchronized void stop() {
        if (running_) {
            System.out.println("Stopping loops");
            notifier_.stop();
            try{
                synchronized (taskRunningLock_) {
                    running_ = false;
                    timestamp_ = Timer.getFPGATimestamp();
                    for (Loop loop : loops_) {
                        System.out.println("Stopping " + loop);
                        loop.onStop(timestamp_);
                    }
                }
            }  catch(Exception e){
                System.out.println("Crash occured on startup of loop");
                handleCrash(e);
            } 
        }
    }

    private void handleCrash(Exception e){
        System.out.println("Looper crashed!");
        e.printStackTrace();
        System.out.println("The software will now restart");
        System.exit(-1);
    }
}
