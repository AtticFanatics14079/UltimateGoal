package org.firstinspires.ftc.teamcode.Autonomous.RRTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.DashboardUtil;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FollowerPIDTuner extends LinearOpMode {
    public static double DISTANCE = 48; // in

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-DISTANCE / 2, -DISTANCE / 2, 0);

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .forward(DISTANCE)
                    .build();
            drive.followTrajectory(traj);
            TelemetryPacket packet = new TelemetryPacket();

            Pose2d estimate = drive.getPoseEstimate();
            packet.put("x", estimate.getX());
            packet.put("y", estimate.getY());
            packet.put("heading", Math.toDegrees(estimate.getHeading()));
            packet.put("imu", Math.toDegrees(drive.getRawExternalHeading()));

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            //drive.turn(Math.toRadians(90));

            //startPose = traj.end().plus(new Pose2d(0, 0, Math.toRadians(90)));
        }
    }
}
