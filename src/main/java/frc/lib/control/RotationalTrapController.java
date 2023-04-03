package frc.lib.control;

import edu.wpi.first.wpilibj.Timer;

public class RotationalTrapController {
    
    public enum RTCState {
        DISABLE,
        ACCEL,
        CRUISING,
        DECEL,
        HOLD
    }

    private double lastUpdate;
    private double maxOmega;
    private int direction;
    private double maxAlpha;
    private double currOmgea;
    private double currTheta;
    private double goalTheta;
    private double kP;
    private double thetaThreshold;
    private RTCState state;

    public RotationalTrapController(double maxOmega, double maxAlpha, double thetaThreshold, double kP) {
        this.maxOmega = maxOmega;
        this.maxAlpha = maxAlpha;
        this.kP = kP;
        currOmgea = 0;
        currTheta = 0;
        goalTheta = currTheta;
        state = RTCState.DISABLE;
        direction = 0;
    }

    public void enableToGoal(double currHeading, double currTime, double goalHeading) {
        goalTheta = goalHeading;
        currTheta = currHeading;
        lastUpdate = currTime;
        state = RTCState.ACCEL;
        direction = ((currTheta - goalTheta) > 0) ? -1 : 1;
    }

    public void disableController() {
        state = RTCState.DISABLE;
    }

    public double updateController(double updateHeading, double updateTime) {
        if(state != RTCState.DISABLE) {
            double dt = updateTime - lastUpdate;
            currTheta = updateHeading;
            switch(state) {
                case ACCEL:
                    currOmgea += (direction * maxAlpha * dt);
                    break;
                case DECEL:
                    currOmgea -= (direction * maxAlpha * dt);
                    break;
                case HOLD:
                    currOmgea = kP * (currTheta - goalTheta);
                    break;
                default:
            }
            lastUpdate = updateTime;
            checkTransition();
        } else {
            System.out.println("ERROR: RTC is disabled!");
        }
        return currOmgea;
    }

    private void checkTransition() {
        switch (state) {
            case ACCEL:
                if(Math.abs(currOmgea * currOmgea / (2 * maxAlpha)) >= Math.abs(goalTheta - currTheta)) {
                    state = RTCState.DECEL;
                } else if(Math.abs(currOmgea) > Math.abs(maxOmega)) {
                    state = RTCState.CRUISING;
                }
                break;
            case CRUISING:
                if(Math.abs(currOmgea * currOmgea / (2 * maxAlpha)) >= Math.abs(goalTheta - currTheta)) {
                    state = RTCState.DECEL;
                }
                break;
            case DECEL:
                if(Math.abs(goalTheta - currTheta) < thetaThreshold) {
                    state = RTCState.HOLD;
                }
                break;
            default:
        }
    }

    public RTCState getState() {
        return state;
    }

}
