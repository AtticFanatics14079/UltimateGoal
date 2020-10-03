package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

@Autonomous
public class GoToPositionOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive odometry = new SampleMecanumDrive(hardwareMap);
        RobotMovement drive = new RobotMovement(hardwareMap);
        MultipleTelemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(!isStopRequested()){
            odometry.update();
            Pose2d currentPose = odometry.getPoseEstimate();
            Point targetPoint = new Point(2,2);
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            drive.goToPosition(24,0,0.3,270,1);
            t.addData("Current Position (Odo): ", currentPose);
            t.addData("Target Point: ", targetPoint);
            t.update();
        }
    }
}
