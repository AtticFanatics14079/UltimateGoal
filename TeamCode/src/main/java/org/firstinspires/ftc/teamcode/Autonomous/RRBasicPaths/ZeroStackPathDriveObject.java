package org.firstinspires.ftc.teamcode.Autonomous.RRBasicPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class ZeroStackPathDriveObject extends LinearOpMode {
    private Pose2d startPose = new Pose2d(-63.0, -36.0, Math.toRadians(0));
    private Pose2d shootPose = new Pose2d(-2.0, -32.0, Math.toRadians(0));
    private Vector2d dropoff0 = new Vector2d(10.0, -60.0);
    private Vector2d pickup = new Vector2d(-52.0, -48.0);

    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;

    @Override
    public void runOpMode() throws InterruptedException {
        ConfigurationRR drive = new ConfigurationRR(hardwareMap);
        hardware = new HardwareThread(hardwareMap, vals, drive);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        hardware.start();

        if(isStopRequested()) return;

        drive.setPoseEstimate(startPose);

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