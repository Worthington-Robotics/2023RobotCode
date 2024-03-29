package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void resetDriveEncoder();

    double getDriveEncoder();

    double getDesiredSteerAngle();
}
