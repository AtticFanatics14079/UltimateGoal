package org.firstinspires.ftc.teamcode.Autonomous.RoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.List;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class AutonomousPaths extends LinearOpMode {
    public static double DISTANCE = 36; // in
    private DriveConstraints constraints = new DriveConstraints(45.0, 30.0, 0.0, Math.toRadians(270), Math.toRadians(270), 0.0);
    private Pose2d startPose = new Pose2d(-63.0, -36.0, Math.toRadians(180));
    private Pose2d shootPose0 = new Pose2d(-2.0, -15.0, Math.toRadians(0));
    private Pose2d shootPose1 = new Pose2d(-2.0, -15.0, Math.toRadians(180));
    private Vector2d dropoff0 = new Vector2d(10.0, -58.0);
    private Vector2d dropoff1 = new Vector2d(34.0, -24.0);
    private Vector2d dropoff2 = new Vector2d(58.0, -48.0);
    private Vector2d pickup = new Vector2d(-48.0, -48.0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TrajectoryBuilder shootergobrr0 = new TrajectoryBuilder(startPose, startPose.getHeading(), constraints);
        TrajectoryBuilder builder0 = new TrajectoryBuilder(shootPose0, 0.0, constraints);
        TrajectoryBuilder builder0point1 = new TrajectoryBuilder(new Pose2d(pickup, Math.toRadians(180)), Math.toRadians(0), constraints);
        TrajectoryBuilder shootergobrr1 = new TrajectoryBuilder(startPose, startPose.getHeading(), constraints);
        TrajectoryBuilder builder1 = new TrajectoryBuilder(shootPose1, true, constraints);
        TrajectoryBuilder builder1point1 = new TrajectoryBuilder(new Pose2d(new Vector2d(dropoff1.getX(), dropoff1.getY() - 6), Math.toRadians(180)), false, constraints);
        TrajectoryBuilder builder1point2 = new TrajectoryBuilder(new Pose2d(pickup, Math.toRadians(180)), true, constraints);
        TrajectoryBuilder builder1point3 = new TrajectoryBuilder(new Pose2d(dropoff1, Math.toRadians(180)), false, constraints);
        TrajectoryBuilder shootergobrr2 = new TrajectoryBuilder(startPose, startPose.getHeading(), constraints);
        TrajectoryBuilder builder2 = new TrajectoryBuilder(shootPose1, true, constraints);
        TrajectoryBuilder builder2point1 = new TrajectoryBuilder(new Pose2d(new Vector2d(dropoff2.getX(), dropoff2.getY() - 6), Math.toRadians(180)), false, constraints);
        TrajectoryBuilder builder2point2 = new TrajectoryBuilder(new Pose2d(pickup, Math.toRadians(180)), true, constraints);
        TrajectoryBuilder builder2point3 = new TrajectoryBuilder(new Pose2d(dropoff2, Math.toRadians(180)), false, constraints);

        shootergobrr0
                .lineToLinearHeading(shootPose0);

        builder0
                .splineTo(dropoff0, Math.toRadians(180))
                .splineTo(pickup, Math.toRadians(180));
        //.splineTo(Vector2d(52.0, -42.0), -90.0.toRadians)
        //.splineTo(Vector2d(-28.0, -32.0), 130.0.toRadians)
        // .splineTo(Vector2d(5.0, -20.0), 0.0.toRadians)

        builder0point1
                .splineTo(new Vector2d(dropoff0.getX(), dropoff0.getY()+6), Math.toRadians(0));

        shootergobrr1
                .lineToLinearHeading(shootPose1);

        builder1
                .splineTo(new Vector2d(dropoff1.getX(), dropoff1.getY() - 6), Math.toRadians(0));
        //.splineTo(Vector2d(52.0, -42.0), -90.0.toRadians)
        //.splineTo(Vector2d(-28.0, -32.0), 130.0.toRadians)
        // .splineTo(Vector2d(5.0, -20.0), 0.0.toRadians)

        builder1point1
                .splineTo(pickup, Math.toRadians(180));

        builder1point2
                .splineTo(dropoff1, Math.toRadians(0));

        builder1point3
                .splineTo(new Vector2d(10.0, dropoff1.getY()), Math.toRadians(180));

        shootergobrr2
                .lineToLinearHeading(shootPose1);

        builder2
                .splineTo(new Vector2d(dropoff2.getX(), dropoff2.getY() - 6), Math.toRadians(0));
        //.splineTo(Vector2d(52.0, -42.0), -90.0.toRadians)
        //.splineTo(Vector2d(-28.0, -32.0), 130.0.toRadians)
        // .splineTo(Vector2d(5.0, -20.0), 0.0.toRadians)

        builder2point1
                .splineTo(pickup, Math.toRadians(180));

        builder2point2
                .splineTo(dropoff2, Math.toRadians(0));

        builder2point3
                .splineTo(new Vector2d(10.0, dropoff2.getY()), Math.toRadians(180));

        //Trajectory shbrr0 = shootergobrr0.build();
        //Trajectory bld0 = builder0.build();
        //Trajectory bld0pnt1 = builder0point1.build();
        Trajectory shbrr1 = shootergobrr1.build();
        Trajectory bld1 = builder1.build();
        Trajectory bld1pnt1 = builder1point1.build();
        Trajectory bld1pnt2 = builder1point2.build();
        Trajectory bld1pnt3 = builder1point3.build();
        //Trajectory shbrr1 = shootergobrr1.build();
        //Trajectory bld2 = builder2.build();
        //Trajectory bld2pnt1 = builder2point1.build();
        //Trajectory bld2pnt2 = builder2point2.build();
        //Trajectory bld2pnt3 = builder2point3.build();

        waitForStart();
        if (isStopRequested()) return;
        while(!isStopRequested()){
            drive.followTrajectory(shbrr1);
            drive.followTrajectory(bld1);
            drive.followTrajectory(bld1pnt1);
            drive.followTrajectory(bld1pnt2);
            drive.followTrajectory(bld1pnt3);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
}