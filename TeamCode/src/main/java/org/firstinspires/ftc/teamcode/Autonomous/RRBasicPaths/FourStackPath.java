package org.firstinspires.ftc.teamcode.Autonomous.RRBasicPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FourStackPath extends LinearOpMode {

    private DriveConstraints constraints = new DriveConstraints(45.0, 30.0, 0.0, Math.toRadians(270), Math.toRadians(270), 0.0);
    private Pose2d startPose = new Pose2d(-63.0, -36.0, Math.toRadians(180.0));
    private Pose2d shootPose1 = new Pose2d(-2.0, -15.0, Math.toRadians(180.0));
    private Vector2d dropoff2 = new Vector2d(58.0, -48.0);
    private Vector2d pickup = new Vector2d(-48.0, -48.0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if(isStopRequested()) return;
        Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootPose1)
                .build();
        drive.followTrajectory(goToShoot);

        //Shoot

        Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(dropoff2.getX(), dropoff2.getY() - 6), Math.toRadians(0.0))
                .addDisplacementMarker(() ->{
                    //do stuff
                })
                .build();
        drive.followTrajectory(wobble1);

        //do stuff

        Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(pickup.getX() + 24, pickup.getY() + 9), Math.toRadians(225.0))
                .splineTo(pickup, Math.toRadians(190.0))
                .addDisplacementMarker(() -> {
                    //do stuff
                })
                .build();
        drive.followTrajectory(wobble2);

        Trajectory shootdrop = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(shootPose1.getX() -22, shootPose1.getY() - 4), Math.toRadians(20.0))
                .splineTo(new Vector2d(shootPose1.getX(), shootPose1.getY()), Math.toRadians(0.0))
                .splineTo(dropoff2, Math.toRadians(0.0))
                .build();
        drive.followTrajectory(shootdrop);

        Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(10.0, dropoff2.getY()), Math.toRadians(180.0))
                .build();
        drive.followTrajectory(toPark);

    }
}