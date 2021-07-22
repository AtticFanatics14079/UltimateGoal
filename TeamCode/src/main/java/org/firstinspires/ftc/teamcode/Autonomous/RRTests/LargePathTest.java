package org.firstinspires.ftc.teamcode.Autonomous.RRTests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class LargePathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ElapsedTime time = new ElapsedTime();

        drive.setPoseEstimate(new Pose2d(-133, -19));

        waitForStart();

        if (isStopRequested()) return;

        double startTime = time.time();
        System.out.println(startTime);

        Trajectory spline1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-28, -32), -Math.toRadians(90))
                .build();
        drive.followTrajectory(spline1);

        Trajectory spline2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-110, -42), -Math.toRadians(90))
                .build();
        drive.followTrajectory(spline2);

        Trajectory spline3 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(-44, -65), -Math.toRadians(90))
                .build();
        drive.followTrajectory(spline3);

        double endTime = time.time();
        System.out.println(endTime);
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.addData("Elapsed Time: ", endTime - startTime);
        telemetry.update();

        System.out.println(endTime - startTime);

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
