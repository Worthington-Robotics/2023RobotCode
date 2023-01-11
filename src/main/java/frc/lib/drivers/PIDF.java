package frc.lib.drivers;

import java.util.concurrent.locks.ReentrantLock;

import frc.lib.util.Util;

public class PIDF {

    //dT is loop time in seconds
    protected double kP = 0, kI = 0, kD = 0, kF = 0, dT = 0, iMax = 0;

    protected double error, lastError, derivative, integral, inputRange, setPoint;
    protected boolean continuous, enabled;
    protected ReentrantLock calculationMutex;

    protected static final double DEFAULT_DT = 0.010;

    public PIDF(double kP, double kD) {
        this(kP, 0, kD, 0, DEFAULT_DT, 0);
    }

    public PIDF(double kP, double kI, double kD, double iMax) {
        this(kP, kI, kD, 0, DEFAULT_DT, iMax);
    }

    /**
     * Complete PIDF constructor
     *
     * @param kP
     * @param kI
     * @param kD
     * @param kF   Not yet implemented
     * @param dT   discrete time step for control loop
     * @param iMax maximum integral windup
     */
    public PIDF(double kP, double kI, double kD, double kF, double dT, double iMax) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.dT = dT;
        this.iMax = iMax;
        calculationMutex = new ReentrantLock();
        reset();
    }

    /**
     * Resets values internal to the PID back to their
     * default states. Also disables the PID if it was currently
     * in use within the software
     */
    public void reset() {
        calculationMutex.lock();
        try {
            error = lastError = derivative = integral = setPoint = 0;
            continuous = enabled = false;
        } finally {
            calculationMutex.unlock();
        }
    }

    /**
     * Allows the PID controller to return a nonzero output
     */
    public void enable() {
        enabled = true;
    }

    /**
     * Stops the PID from returning a nonzero output
     */
    public void disable() {
        enabled = false;
    }

    public void setPoint(double setPoint) {
        calculationMutex.lock();
        try {
            this.setPoint = setPoint;
        } finally {
            calculationMutex.unlock();
        }
    }

    /**
     * Sets the P, I and D gains
     *
     * @param kP
     * @param kI
     * @param kD
     */
    public void setPID(double kP, double kI, double kD) {
        setPIDF(kP, kI, kD, kF);
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        calculationMutex.lock();
        try {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
        } finally {
            calculationMutex.unlock();
        }
    }

    public double[] getPID(){
        double[] out = new double[4];
        calculationMutex.lock();
        try {
            out[0] = this.kP;
            out[1] = this.kI;
            out[2] = this.kD;
            out[3] = this.kF;
        } finally {
            calculationMutex.unlock();
        }
        return out;
    }

    /**
     * Returns the current calculated output of this PID
     * based on the current position
     *
     * @param input system position
     * @return calculated motor power to apply (%)
     */
    public double update(double input) {
        if (!enabled) {
            return 0.0;
        }

        final double result;
        calculationMutex.lock();

        try {
            // calculate p term
            error = getContinuousError(setPoint - input);

            // calculate D term based on previous P term
            derivative = (error - lastError) / dT;

            // calculate and bound I term to iMax
            integral += error * dT;
            if (Math.abs(integral) > iMax) {
                integral = Math.signum(integral) * iMax;
            }

            // save last error for d term
            lastError = error;

            result = error * kP + integral * kI + derivative * kD;
        } finally {
            calculationMutex.unlock();
        }

        return clamp(result, -1, 1);
    }

    /**
     * Wraps error around for continuous inputs. The original
     * error is returned if continuous mode is disabled.
     *
     * @param error The current error of the PID controller.
     * @return Error for continuous inputs.
     */
    protected double getContinuousError(double error) {
        if (continuous && inputRange > 0) {
            error %= inputRange;
            if (Math.abs(error) > inputRange / 2) {
                if (error > 0) {
                    return error - inputRange;
                } else {
                    return error + inputRange;
                }
            }
        }

        return error;
    }

    public double getError(){
        calculationMutex.lock();
        try {
            return error;
        } finally {
            calculationMutex.unlock();
        }
    }

    /**
     * Sets the PID to use a continuous mode calculation that
     * makes both ends of the range effectively the same point
     * !!!YOU MUST MAKE SURE TO SET THE RANGE IF YOU USE THIS MODE!!!
     *
     * @param continuous
     */
    public void setContinuous(boolean continuous) {
        calculationMutex.lock();
        try {
            this.continuous = continuous;
        } finally {
            calculationMutex.unlock();
        }
    }

    /**
     * Sets the min am maxium values defining the input range for this
     * PID controller. Only needed for continuous PIDs
     *
     * @param minimumInput
     * @param maximumInput
     */
    public void setInputRange(double minimumInput, double maximumInput) {
        if (maximumInput < minimumInput) {
            throw new RuntimeException("Attempted to set a maximum input range lesser than the minimum");
        }

        calculationMutex.lock();
        try {
            inputRange = maximumInput - minimumInput;
        } finally {
            calculationMutex.unlock();
        }
    }

    public boolean onTarget(double epsilon) {
        return Util.epsilonEquals(error, 0, epsilon);
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
    }
}