package com.pathplanner.lib.actions;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.statemachine.Action;

public class PPSwerveControllerAction extends Action {
    private final Timer timer = new Timer();
    private final PathPlannerTrajectory trajectory;
    private final Supplier<Pose2d> poseSupplier;
    private final SwerveDriveKinematics kinematics;
     private final PPHolonomicDriveController controller;
    private final Consumer<SwerveModuleState[]> outputModuleStates;
    private final Consumer<ChassisSpeeds> outputChassisSpeeds;
    private final boolean useKinematics;
    private final boolean useAllianceColor;

    private PathPlannerTrajectory transformedTrajectory;

    private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
    private static Consumer<Pose2d> logTargetPose = null;
    private static Consumer<ChassisSpeeds> logSetpoint = null;
     private static BiConsumer<Translation2d, Rotation2d> logError = PPSwerveControllerAction::defaultLogError;

       /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but ChassisSpeeds from the position
   * controllers which need to be converted to module states and put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputChassisSpeeds The field relative chassis speeds output consumer.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  public PPSwerveControllerAction(
    PathPlannerTrajectory trajectory,
    Supplier<Pose2d> poseSupplier,
    PIDController xController,
    PIDController yController,
    PIDController rotationController,
    Consumer<ChassisSpeeds> outputChassisSpeeds,
    boolean useAllianceColor) {
  this.trajectory = trajectory;
  this.poseSupplier = poseSupplier;
  this.controller = new PPHolonomicDriveController(xController, yController, rotationController);
  this.outputChassisSpeeds = outputChassisSpeeds;
  this.outputModuleStates = null;
  this.kinematics = null;
  this.useKinematics = false;
  this.useAllianceColor = useAllianceColor;

  if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
    DriverStation.reportWarning(
        "You have constructed a path following command that will automatically transform path states depending"
            + " on the alliance color, however, it appears this path was created on the red side of the field"
            + " instead of the blue side. This is likely an error.",
        false);
  }
}

    @Override
    public void onStart() {
        if (useAllianceColor && trajectory.fromGUI) {
            transformedTrajectory =
                PathPlannerTrajectory.transformTrajectoryForAlliance(
                    trajectory, DriverStation.getAlliance());
          } else {
            transformedTrajectory = trajectory;
          }
      
          if (logActiveTrajectory != null) {
            logActiveTrajectory.accept(transformedTrajectory);
          }
      
          timer.reset();
          timer.start();
      
          PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
    }

    @Override
    public void onLoop() {
        double currentTime = this.timer.get();
        PathPlannerState desiredState = (PathPlannerState) transformedTrajectory.sample(currentTime);
    
        Pose2d currentPose = this.poseSupplier.get();
    
        PathPlannerServer.sendPathFollowingData(
            new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
            currentPose);
    
        ChassisSpeeds targetChassisSpeeds = this.controller.calculate(currentPose, desiredState);
    
        if (this.useKinematics) {
          SwerveModuleState[] targetModuleStates =
              this.kinematics.toSwerveModuleStates(targetChassisSpeeds);
    
          this.outputModuleStates.accept(targetModuleStates);
        } else {
          this.outputChassisSpeeds.accept(targetChassisSpeeds);
        }
    
        if (logTargetPose != null) {
          logTargetPose.accept(
              new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation));
        }
    
        if (logError != null) {
          logError.accept(
              currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
              currentPose.getRotation().minus(desiredState.holonomicRotation));
        }
    
        if (logSetpoint != null) {
          logSetpoint.accept(targetChassisSpeeds);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void onStop() {
    this.timer.stop();
    }

    private static void defaultLogError(Translation2d translationError, Rotation2d rotationError) {
        SmartDashboard.putNumber("PPSwerveControllerAction/xErrorMeters", translationError.getX());
        SmartDashboard.putNumber("PPSwerveControllerAction/yErrorMeters", translationError.getY());
        SmartDashboard.putNumber(
            "PPSwerveControllerAction/rotationErrorDegrees", rotationError.getDegrees());
      }
    
}
