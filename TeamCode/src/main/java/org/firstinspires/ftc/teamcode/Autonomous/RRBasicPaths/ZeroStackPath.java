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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Disabled
@Config
@Autonomous(group = "drive")
public class ZeroStackPath extends LinearOpMode {
    private Pose2d startPose = new Pose2d(-63.0, -36.0, Math.toRadians(0));
    private Pose2d shootPose = new Pose2d(-2.0, -32.0, Math.toRadians(0));
    private Vector2d dropoff0 = new Vector2d(10.0, -60.0);
    private Vector2d pickup = new Vector2d(-52.0, -48.0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.update();

        waitForStart();

        if(isStopRequested()) return;

        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.update();

        //Should fix it.
        drive.setPoseEstimate(startPose);

        /*Trajectory test = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-50, -20), Math.toRadians(90))
                .build();
        drive.followTrajectory(test);

         */

        Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(shootPose.getX(), shootPose.getY()), 0)
                .build();
        drive.followTrajectory(goToShoot);

        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.update();

        Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(dropoff0, Math.toRadians(180))
                .addTemporalMarker(1, () ->{
                    telemetry.addData("Position: ", drive.getPoseEstimate());
                    telemetry.update();
                })
                .splineTo(pickup, Math.toRadians(180))
                .build();
        drive.followTrajectory(wobble1);

        //do stuff

        Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                .splineTo(new Vector2d(dropoff0.getX() + 6, dropoff0.getY()+8), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    //do stuff
                })
                .build();
        drive.followTrajectory(wobble2);

    }
}