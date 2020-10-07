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
public class ZeroStackPath extends LinearOpMode {
    private Pose2d startPose = new Pose2d(-63.0, -36.0, Math.toRadians(180));
    private Pose2d shootPose = new Pose2d(-2.0, -15.0, Math.toRadians(0));
    private Vector2d dropoff0 = new Vector2d(10.0, -58.0);
    private Vector2d pickup = new Vector2d(-48.0, -48.0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        if(isStopRequested()) return;
        Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(shootPose)
                .build();
        drive.followTrajectory(goToShoot);

        Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(dropoff0, Math.toRadians(180))
                .addDisplacementMarker(() ->{
                    //do stuff
                })
                .splineTo(pickup, Math.toRadians(180))
                .build();
        drive.followTrajectory(wobble1);

        //do stuff

        Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(dropoff0.getX(), dropoff0.getY()+6), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //do stuff
                })
                .build();
        drive.followTrajectory(wobble2);
    }
}