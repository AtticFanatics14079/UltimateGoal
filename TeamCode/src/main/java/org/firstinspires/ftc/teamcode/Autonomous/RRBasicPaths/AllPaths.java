package org.firstinspires.ftc.teamcode.Autonomous.RRBasicPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.SampleConfiguration;

public class AllPaths extends LinearOpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    private DriveConstraints constraints = new DriveConstraints(45.0, 30.0, 0.0, Math.toRadians(270), Math.toRadians(270), 0.0);
    private Pose2d startPose = new Pose2d(-63.0, -36.0, Math.toRadians(180.0));
    private Pose2d shootPose0 = new Pose2d(-2.0, -15.0, Math.toRadians(0.0));
    private Pose2d shootPose1 = new Pose2d(-2.0, -15.0, Math.toRadians(180.0));
    private Vector2d dropoff0 = new Vector2d(10.0, -58.0);
    private Vector2d dropoff1 = new Vector2d(34.0, -24.0);
    private Vector2d dropoff2 = new Vector2d(58.0, -48.0);
    private Vector2d pickup = new Vector2d(-48.0, -48.0);



    @Override
    public void runOpMode() throws InterruptedException {

        TrajectoryBuilder shootergobrr0 = new TrajectoryBuilder(startPose, startPose.getHeading(), constraints);

        TrajectoryBuilder builder0 = new TrajectoryBuilder(shootPose0, 0.0, constraints);

        TrajectoryBuilder builder0point1 = new TrajectoryBuilder(new Pose2d(pickup, Math.toRadians(180.0)), Math.toRadians(0.0), constraints);

        TrajectoryBuilder shootergobrr1 = new TrajectoryBuilder(startPose, startPose.getHeading(), constraints);

        TrajectoryBuilder builder1 = new TrajectoryBuilder(shootPose1, true, constraints);

        TrajectoryBuilder builder1point1 = new TrajectoryBuilder(new Pose2d(new Vector2d(dropoff1.getX(), dropoff1.getY() - 6), Math.toRadians(180.0)), false, constraints);

        TrajectoryBuilder builder1point2 = new TrajectoryBuilder(new Pose2d(pickup, Math.toRadians(210.0)), true, constraints);

        TrajectoryBuilder builder1point3 = new TrajectoryBuilder(new Pose2d(dropoff1, Math.toRadians(180.0)), false, constraints);

        TrajectoryBuilder shootergobrr2 = new TrajectoryBuilder(startPose, startPose.getHeading(), constraints);

        TrajectoryBuilder builder2 = new TrajectoryBuilder(shootPose1, true, constraints);

        TrajectoryBuilder builder2point1 = new TrajectoryBuilder(new Pose2d(new Vector2d(dropoff2.getX(), dropoff2.getY() - 6), Math.toRadians(180.0)), false, constraints);

        TrajectoryBuilder builder2point2 = new TrajectoryBuilder(new Pose2d(pickup, Math.toRadians(190.0)), true, constraints);

        TrajectoryBuilder builder2point3 = new TrajectoryBuilder(new Pose2d(dropoff2, Math.toRadians(180.0)), false, constraints);

        shootergobrr0
                .lineToLinearHeading(shootPose0);

        builder0
                .splineTo(dropoff0, Math.toRadians(0))
                .splineTo(pickup, Math.toRadians(180.0));
        //.splineTo(Vector2d(52.0, -42.0), -90.0.toRadians)
        //.splineTo(Vector2d(-28.0, -32.0), 130.0.toRadians)
        // .splineTo(Vector2d(5.0, -20.0), 0.0.toRadians)

        builder0point1
        .splineTo(new Vector2d(dropoff0.getX(), dropoff0.getY()+6), Math.toRadians(0.0));

        shootergobrr1
                .lineToLinearHeading(shootPose1);

        builder1
                .splineTo(new Vector2d(dropoff1.getX(), dropoff1.getY() - 6), Math.toRadians(0.0));
        //.splineTo(Vector2d(52.0, -42.0), -90.0.toRadians)
        //.splineTo(Vector2d(-28.0, -32.0), 130.0.toRadians)
        // .splineTo(Vector2d(5.0, -20.0), 0.0.toRadians)

        builder1point1
                .splineTo(pickup, Math.toRadians(210.0));
        builder1point2
                .splineTo(new Vector2d(shootPose1.getX(), shootPose1.getY()), Math.toRadians(0.0))
                .splineTo(dropoff1, Math.toRadians(0.0));
        builder1point3
                .splineTo(new Vector2d(10.0, dropoff1.getY()), Math.toRadians(180.0));

        shootergobrr2
                .lineToLinearHeading(shootPose1);

        builder2
                .splineTo(new Vector2d(dropoff2.getX(), dropoff2.getY() - 6), Math.toRadians(0.0));
        //.splineTo(Vector2d(52.0, -42.0), -90.0.toRadians)
        //.splineTo(Vector2d(-28.0, -32.0), 130.0.toRadians)
        // .splineTo(Vector2d(5.0, -20.0), 0.0.toRadians)

        builder2point1
                .splineTo(new Vector2d(pickup.getX() + 24, pickup.getY() + 9), Math.toRadians(225.0))
                .splineTo(pickup, Math.toRadians(190.0));

        builder2point2
                .splineTo(new Vector2d(shootPose1.getX() -22, shootPose1.getY() - 4), Math.toRadians(20.0))
                .splineTo(new Vector2d(shootPose1.getX(), shootPose1.getY()), Math.toRadians(0.0))
                .splineTo(dropoff2, Math.toRadians(0.0));

        builder2point3
                .splineTo(new Vector2d(10.0, dropoff2.getY()), Math.toRadians(180.0));

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
    }
}





