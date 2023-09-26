package frc.robot.subsystems.arm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.arm.ArmTrajectory.Parameters;

public class ArmTrajectoryManager {
    private List<ArmTrajectory> trajectories = new ArrayList<>();
    private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(3, 2).setStartVelocity(0).setEndVelocity(0);

    public ArmTrajectoryManager() {
        System.out.println("Generating splines.");

        //Zero to Transit and Transit to Zero
        List<Translation2d> firstWaypoints = new ArrayList<>();
        Trajectory firstTraj = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.ZERO.getPose2d(), firstWaypoints, ArmPose.Preset.TRANSIT.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(firstTraj.sample(0).poseMeters, firstTraj.sample(firstTraj.getTotalTimeSeconds()).poseMeters, firstTraj));
        Trajectory secondTraj = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.TRANSIT.getPose2d(), firstWaypoints, ArmPose.Preset.ZERO.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(secondTraj.sample(0).poseMeters, secondTraj.sample(secondTraj.getTotalTimeSeconds()).poseMeters, secondTraj));

        //Transit to Mid, Mid to Transit
        List<Translation2d> thirdWaypoints = new ArrayList<>();
        Trajectory thirdTraj = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.TRANSIT.getPose2d(), thirdWaypoints, ArmPose.Preset.MID.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirdTraj.sample(0).poseMeters, thirdTraj.sample(thirdTraj.getTotalTimeSeconds()).poseMeters, thirdTraj));
        Trajectory fourTraj = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.MID.getPose2d(), thirdWaypoints, ArmPose.Preset.TRANSIT.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fourTraj.sample(0).poseMeters, fourTraj.sample(fourTraj.getTotalTimeSeconds()).poseMeters, fourTraj));

        //Zero to Intake, Intake to Zero
        List<Translation2d> fifthWaypoints = new ArrayList<>();
        Trajectory fifthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.ZERO.getPose2d(), fifthWaypoints, ArmPose.Preset.INTAKE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fifthTrajectory.sample(0).poseMeters, fifthTrajectory.sample(fifthTrajectory.getTotalTimeSeconds()).poseMeters, fifthTrajectory));
        Trajectory sixthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.INTAKE.getPose2d(), fifthWaypoints, ArmPose.Preset.ZERO.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(sixthTrajectory.sample(0).poseMeters, sixthTrajectory.sample(sixthTrajectory.getTotalTimeSeconds()).poseMeters, sixthTrajectory));

        //Zero to High, High to Zero
        List<Translation2d> seventhWaypoints = new ArrayList<>();
        Trajectory seventhTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.ZERO.getPose2d(), seventhWaypoints, ArmPose.Preset.HIGH.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(seventhTrajectory.sample(0).poseMeters, seventhTrajectory.sample(seventhTrajectory.getTotalTimeSeconds()).poseMeters, seventhTrajectory));
        Trajectory eigthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.HIGH.getPose2d(), seventhWaypoints, ArmPose.Preset.ZERO.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(eigthTrajectory.sample(0).poseMeters, eigthTrajectory.sample(eigthTrajectory.getTotalTimeSeconds()).poseMeters, eigthTrajectory));

        //Zero to Slide, Slide to Zero
        List<Translation2d> ninethWaypoints = new ArrayList<>();
        Trajectory ninethTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.ZERO.getPose2d(), ninethWaypoints, ArmPose.Preset.SLIDE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(ninethTrajectory.sample(0).poseMeters, ninethTrajectory.sample(ninethTrajectory.getTotalTimeSeconds()).poseMeters, ninethTrajectory));
        Trajectory tenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SLIDE.getPose2d(), ninethWaypoints, ArmPose.Preset.ZERO.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(tenthTrajectory.sample(0).poseMeters, tenthTrajectory.sample(tenthTrajectory.getTotalTimeSeconds()).poseMeters, tenthTrajectory));

        //Transit to High, High to Transit
        List<Translation2d> eleventhWaypoint = new ArrayList<>();
        Trajectory eleventhTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.TRANSIT.getPose2d(), eleventhWaypoint, ArmPose.Preset.HIGH.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(eleventhTrajectory.sample(0).poseMeters, eleventhTrajectory.sample(eleventhTrajectory.getTotalTimeSeconds()).poseMeters, eleventhTrajectory));
        Trajectory twelthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.HIGH.getPose2d(), eleventhWaypoint, ArmPose.Preset.TRANSIT.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twelthTrajectory.sample(0).poseMeters, twelthTrajectory.sample(twelthTrajectory.getTotalTimeSeconds()).poseMeters, twelthTrajectory));

        //Transit to Slide, Slide to Transit
        List<Translation2d> thirteenthWaypoint = new ArrayList<>();
        Trajectory thirteenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.TRANSIT.getPose2d(), thirteenthWaypoint, ArmPose.Preset.SLIDE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirteenthTrajectory.sample(0).poseMeters, thirteenthTrajectory.sample(thirteenthTrajectory.getTotalTimeSeconds()).poseMeters, thirteenthTrajectory));
        Trajectory fourteenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SLIDE.getPose2d(), thirteenthWaypoint, ArmPose.Preset.TRANSIT.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fourteenthTrajectory.sample(0).poseMeters, fourteenthTrajectory.sample(fourteenthTrajectory.getTotalTimeSeconds()).poseMeters, fourteenthTrajectory));

        //Transit to Intake, Intake to Transit
        List<Translation2d> fifteenthWaypoint = new ArrayList<>();
        Trajectory fifteenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.TRANSIT.getPose2d(), fifteenthWaypoint, ArmPose.Preset.INTAKE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fifteenthTrajectory.sample(0).poseMeters, fifteenthTrajectory.sample(fifteenthTrajectory.getTotalTimeSeconds()).poseMeters, fifteenthTrajectory));
        Trajectory sixteenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.INTAKE.getPose2d(), fifteenthWaypoint, ArmPose.Preset.TRANSIT.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(sixteenthTrajectory.sample(0).poseMeters, sixteenthTrajectory.sample(sixteenthTrajectory.getTotalTimeSeconds()).poseMeters, sixteenthTrajectory));

        //Transit to Shelf, Shelf to Transit
        List<Translation2d> seventeenthWaypoints = new ArrayList<>();
        Trajectory seventeenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.TRANSIT.getPose2d(), seventeenthWaypoints, ArmPose.Preset.SHELF.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(seventeenthTrajectory.sample(0).poseMeters, seventeenthTrajectory.sample(seventeenthTrajectory.getTotalTimeSeconds()).poseMeters, seventeenthTrajectory));
        Trajectory eighteenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SHELF.getPose2d(), seventeenthWaypoints, ArmPose.Preset.TRANSIT.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(eighteenthTrajectory.sample(0).poseMeters, eighteenthTrajectory.sample(eighteenthTrajectory.getTotalTimeSeconds()).poseMeters, eighteenthTrajectory));

        //Mid to Slide, Slide to Mid
        List<Translation2d> nineteenthWaypoints = new ArrayList<>();
        Trajectory nineteenthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.MID.getPose2d(), nineteenthWaypoints, ArmPose.Preset.SLIDE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(nineteenthTrajectory.sample(0).poseMeters, nineteenthTrajectory.sample(nineteenthTrajectory.getTotalTimeSeconds()).poseMeters, nineteenthTrajectory));
        Trajectory twentythTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SLIDE.getPose2d(), nineteenthWaypoints, ArmPose.Preset.MID.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentythTrajectory.sample(0).poseMeters, twentythTrajectory.sample(twentythTrajectory.getTotalTimeSeconds()).poseMeters, twentythTrajectory));

        //Mid to High, High to Mid
        List<Translation2d> twentyFirstWaypoints = new ArrayList<>();
        Trajectory twentyFirstTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.MID.getPose2d(), twentyFirstWaypoints, ArmPose.Preset.HIGH.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyFirstTrajectory.sample(0).poseMeters, twentyFirstTrajectory.sample(twentyFirstTrajectory.getTotalTimeSeconds()).poseMeters, twentyFirstTrajectory));
        Trajectory twentyTwoTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.HIGH.getPose2d(), twentyFirstWaypoints, ArmPose.Preset.MID.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyTwoTrajectory.sample(0).poseMeters, twentyTwoTrajectory.sample(twentyTwoTrajectory.getTotalTimeSeconds()).poseMeters, twentyTwoTrajectory));

        //Mid to Intake, Intake to Mid
        List<Translation2d> twentyThirdWaypoints = new ArrayList<>();
        Trajectory twentyThirdTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.MID.getPose2d(), twentyThirdWaypoints, ArmPose.Preset.INTAKE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyThirdTrajectory.sample(0).poseMeters, twentyThirdTrajectory.sample(twentyThirdTrajectory.getTotalTimeSeconds()).poseMeters, twentyThirdTrajectory));
        Trajectory twentyFourthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.INTAKE.getPose2d(), twentyThirdWaypoints, ArmPose.Preset.MID.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyFourthTrajectory.sample(0).poseMeters, twentyFourthTrajectory.sample(twentyFourthTrajectory.getTotalTimeSeconds()).poseMeters, twentyFourthTrajectory));

        //Mid to Shelf, Shelf to Mid
        List<Translation2d> twentyFifthWaypoints = new ArrayList<>();
        Trajectory twentyFifthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.MID.getPose2d(), twentyFifthWaypoints, ArmPose.Preset.SHELF.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyFifthTrajectory.sample(0).poseMeters, twentyFifthTrajectory.sample(twentyFifthTrajectory.getTotalTimeSeconds()).poseMeters, twentyFifthTrajectory));
        Trajectory twentySixthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SHELF.getPose2d(), twentyFifthWaypoints, ArmPose.Preset.MID.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentySixthTrajectory.sample(0).poseMeters, twentySixthTrajectory.sample(twentySixthTrajectory.getTotalTimeSeconds()).poseMeters, twentySixthTrajectory));

        //High to Slide, Slide to High
        List<Translation2d> twentySeventhWaypoints = new ArrayList<>();
        Trajectory twentySeventhTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.HIGH.getPose2d(), twentySeventhWaypoints, ArmPose.Preset.SLIDE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentySeventhTrajectory.sample(0).poseMeters, twentySeventhTrajectory.sample(twentySeventhTrajectory.getTotalTimeSeconds()).poseMeters, twentySeventhTrajectory));
        Trajectory twentyEighthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SLIDE.getPose2d(), twentySeventhWaypoints, ArmPose.Preset.HIGH.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyEighthTrajectory.sample(0).poseMeters, twentyEighthTrajectory.sample(twentyEighthTrajectory.getTotalTimeSeconds()).poseMeters, twentySixthTrajectory));

        //High to Intake, Intake to High
        List<Translation2d> twentyNinthWaypoints = new ArrayList<>();
        Trajectory twentyNinthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.HIGH.getPose2d(), twentyNinthWaypoints, ArmPose.Preset.INTAKE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(twentyNinthTrajectory.sample(0).poseMeters, twentyNinthTrajectory.sample(twentyNinthTrajectory.getTotalTimeSeconds()).poseMeters, twentyNinthTrajectory));
        Trajectory thirtythTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.INTAKE.getPose2d(), twentyNinthWaypoints, ArmPose.Preset.HIGH.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtythTrajectory.sample(0).poseMeters, thirtythTrajectory.sample(thirtythTrajectory.getTotalTimeSeconds()).poseMeters, thirtythTrajectory));

        //High to Shelf, Shelf to High
        List<Translation2d> thirtyFirsthWaypoints = new ArrayList<>();
        Trajectory thirtyFirstTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.HIGH.getPose2d(), thirtyFirsthWaypoints, ArmPose.Preset.SHELF.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtyFirstTrajectory.sample(0).poseMeters, thirtyFirstTrajectory.sample(thirtyFirstTrajectory.getTotalTimeSeconds()).poseMeters, thirtyFirstTrajectory));
        Trajectory thirtySecondTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SHELF.getPose2d(), thirtyFirsthWaypoints, ArmPose.Preset.HIGH.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtySecondTrajectory.sample(0).poseMeters, thirtySecondTrajectory.sample(thirtySecondTrajectory.getTotalTimeSeconds()).poseMeters, thirtySecondTrajectory));

        //Slide to Intake, Intake to Slide
        List<Translation2d> thirtyThirdWaypoints = new ArrayList<>();
        Trajectory thirtyThirdTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SLIDE.getPose2d(), thirtyThirdWaypoints, ArmPose.Preset.INTAKE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtyThirdTrajectory.sample(0).poseMeters, thirtyThirdTrajectory.sample(thirtyThirdTrajectory.getTotalTimeSeconds()).poseMeters, thirtyThirdTrajectory));
        Trajectory thirtyFourthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.INTAKE.getPose2d(), thirtyThirdWaypoints, ArmPose.Preset.SLIDE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtyFourthTrajectory.sample(0).poseMeters, thirtyFourthTrajectory.sample(thirtyFourthTrajectory.getTotalTimeSeconds()).poseMeters, thirtyFourthTrajectory));

        //Slide to Shelf, Shelf to Slide
        List<Translation2d> thirtyFifthWaypoints = new ArrayList<>();
        Trajectory thirtyFifthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SLIDE.getPose2d(), thirtyFifthWaypoints, ArmPose.Preset.SHELF.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtyFifthTrajectory.sample(0).poseMeters, thirtyFifthTrajectory.sample(thirtyFifthTrajectory.getTotalTimeSeconds()).poseMeters, thirtyFifthTrajectory));
        Trajectory thirtySixthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SHELF.getPose2d(), thirtyFifthWaypoints, ArmPose.Preset.SLIDE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtySixthTrajectory.sample(0).poseMeters, thirtySixthTrajectory.sample(thirtySixthTrajectory.getTotalTimeSeconds()).poseMeters, thirtySixthTrajectory));

        //Intake to Shelf, Shelf to Intake
        List<Translation2d> thirtySeventhWaypoints = new ArrayList<>();
        Trajectory thirtySeventhTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.INTAKE.getPose2d(), thirtySeventhWaypoints, ArmPose.Preset.SHELF.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtySeventhTrajectory.sample(0).poseMeters, thirtySeventhTrajectory.sample(thirtySeventhTrajectory.getTotalTimeSeconds()).poseMeters, thirtySeventhTrajectory));
        Trajectory thirtyEigthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SHELF.getPose2d(), thirtySeventhWaypoints, ArmPose.Preset.INTAKE.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtyEigthTrajectory.sample(0).poseMeters, thirtyEigthTrajectory.sample(thirtyEigthTrajectory.getTotalTimeSeconds()).poseMeters, thirtyEigthTrajectory));

        //Zero to Shelf, Shelf to Zero
        List<Translation2d> thirtyNinthWaypoints = new ArrayList<>();
        Trajectory thirtyNinthTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.ZERO.getPose2d(), thirtyNinthWaypoints, ArmPose.Preset.SHELF.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(thirtyNinthTrajectory.sample(0).poseMeters, thirtyNinthTrajectory.sample(thirtyNinthTrajectory.getTotalTimeSeconds()).poseMeters, thirtyNinthTrajectory));
        Trajectory fortythTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.SHELF.getPose2d(), thirtyNinthWaypoints, ArmPose.Preset.ZERO.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fortythTrajectory.sample(0).poseMeters, fortythTrajectory.sample(fortythTrajectory.getTotalTimeSeconds()).poseMeters, fortythTrajectory));

        //Zero to Mid, Mid to Zero
        List<Translation2d> fortyFirstWaypoints = new ArrayList<>();
        Trajectory fortyFirstTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.ZERO.getPose2d(), fortyFirstWaypoints, ArmPose.Preset.MID.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fortyFirstTrajectory.sample(0).poseMeters, fortyFirstTrajectory.sample(fortyFirstTrajectory.getTotalTimeSeconds()).poseMeters, fortyFirstTrajectory));
        Trajectory fortySecondTrajectory = TrajectoryGenerator.generateTrajectory(ArmPose.Preset.MID.getPose2d(), fortyFirstWaypoints, ArmPose.Preset.ZERO.getPose2d(), trajectoryConfig);
        trajectories.add(trajToArmTraj(fortySecondTrajectory.sample(0).poseMeters, fortySecondTrajectory.sample(fortySecondTrajectory.getTotalTimeSeconds()).poseMeters, fortySecondTrajectory));

        // for(ArmPose.Preset pose : ArmPose.Preset.values()) {
        //     List<Preset> leftovPresets = new ArrayList<>(); //Get all other poses that arent this one
        //     for (Preset preset : ArmPose.Preset.values()) {
        //         if (pose != preset) {
        //             leftovPresets.add(preset);
        //         }
        //     }

        //     for(Preset preset : leftovPresets) {
        //         List<Translation2d> waypoints = new ArrayList<>();
        //         Trajectory traj = TrajectoryGenerator.generateTrajectory(pose.getPose2d(), waypoints, preset.getPose2d(), trajectoryConfig);
        //         trajectories.add(trajToArmTraj(pose.getPose2d(), preset.getPose2d(), traj));
        //     }
        // }

        System.out.println("Done generating splines. Total: " + trajectories.size());
    }

    public ArmTrajectory splineToTraj(Pose2d initialPose, Pose2d finalPose, double time, CubicHermiteSpline spline) {
        ArmTrajectory trajectory = new ArmTrajectory(new Parameters(ArmKinematics.inverseSafe(initialPose), ArmKinematics.inverseSafe(finalPose)));

        List<Vector<N3>> points = new ArrayList<>();
        for(double i = 0; i<1.0; i+=0.02) {
            points.add(ArmKinematics.inverseSafe(spline.getPoint(i).poseMeters));
        }
        trajectory.setPoints(time, points);
        return trajectory;
    }

    public ArmTrajectory trajToArmTraj(Pose2d initialPose, Pose2d finalPose, Trajectory traj) {
        ArmTrajectory trajectory = new ArmTrajectory(new Parameters(ArmKinematics.inverseSafe(initialPose), ArmKinematics.inverseSafe(finalPose)));

        List<Vector<N3>> points = new ArrayList<>();
        for(double i = 0; i<traj.getTotalTimeSeconds(); i+=0.02) {
            points.add(ArmKinematics.inverseSafe(traj.sample(i).poseMeters));
        }
        trajectory.setPoints(traj.getTotalTimeSeconds(), points);
        return trajectory;
    }

    public ArmTrajectory getTrajectory(int index) {
        return trajectories.get(index);
    }

    public ArmTrajectory getTrajectory(Pose2d initialPose, Pose2d finalPose) {
        ArmTrajectory returnTrajectory = new ArmTrajectory(null);
        for (ArmTrajectory trajectory : trajectories) {
            if (trajectory.sample(0).equals(ArmKinematics.inverseSafe(initialPose)) && trajectory.sample(trajectory.getTotalTime()).equals(ArmKinematics.inverseSafe(finalPose))) {
                returnTrajectory = trajectory;
            }
        }
        return returnTrajectory;
    }

    public ArmTrajectory getTrajectory(Vector<N3> initialPose, Vector<N3> finalPose) {
        ArmTrajectory returnTrajectory = new ArmTrajectory(null);
        for (ArmTrajectory trajectory : trajectories) {
            if (trajectory.sample(0).equals(initialPose) && trajectory.sample(trajectory.getTotalTime()).equals(finalPose)) {
                returnTrajectory = trajectory;
            }
        }
        return returnTrajectory;
    }
}
