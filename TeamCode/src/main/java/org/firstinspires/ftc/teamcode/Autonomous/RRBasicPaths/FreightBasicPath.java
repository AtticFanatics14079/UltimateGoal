package org.firstinspires.ftc.teamcode.Autonomous.RRBasicPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FreightBasicPath extends LinearOpMode {
    private Pose2d startPose = new Pose2d(-72.0, -36.0, Math.toRadians(180));
    private Pose2d spinPose = new Pose2d(-54.0 , -64.0, Math.toRadians(180));
    private Pose2d allianceGoalPose = new Pose2d(-24.0, -12.0, Math.toRadians(0));
    private Pose2d allianceGoalDropOffPose = new Pose2d(-24.0, -22.0, Math.toRadians(100));
    private Pose2d tapedParkPose = new Pose2d(-36.0,-60.0,90);

    public static double OPEN = 1, CLOSED = 0;

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
        drive.setPoseEstimate(startPose);

        Trajectory goToSpin = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(spinPose)
                .build();
        drive.followTrajectory(goToSpin);

        Trajectory touchSpin = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(3, new DriveConstraints(20, 10, 0.0, Math.toRadians(140.0), Math.toRadians(140.0), 0.0))
                .build();
        drive.followTrajectory(touchSpin);

        drive.spinner.setPower(-0.4);
        sleep(5000);

        Trajectory goToDropOff = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(allianceGoalDropOffPose.getX(), allianceGoalDropOffPose.getY()), 0)
                .build();
        drive.followTrajectory(goToDropOff);

        drive.slides.setTargetPosition(500);
        drive.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.slides.setPower(1);
        while(Math.abs(drive.slides.getCurrentPosition() - 500) > 50){}
        drive.dropper.setPosition(OPEN);
        sleep(600);

        Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(tapedParkPose.vec(), 0)
                .build();
        drive.followTrajectory(toPark);

        drive.dropper.setPosition(CLOSED);
        drive.slides.setTargetPosition(100);
        drive.slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.slides.setPower(-1);
        sleep(3000);
    }
}