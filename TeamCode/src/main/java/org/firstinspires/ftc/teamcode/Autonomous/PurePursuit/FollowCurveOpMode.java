package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous
public class FollowCurveOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive odometry = new SampleMecanumDrive(hardwareMap);
        RobotMovement drive = new RobotMovement(hardwareMap);
        MultipleTelemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ArrayList<CurvePoint> aNtWoNsMoLbRaIn = new ArrayList<>();
        aNtWoNsMoLbRaIn.add(new CurvePoint(0, 0, 0.5, 0.5, 15, Math.toRadians(50), 1));
        aNtWoNsMoLbRaIn.add(new CurvePoint(0, 36, 0.5, 0.5, 15, Math.toRadians(50), 1));
        aNtWoNsMoLbRaIn.add(new CurvePoint(36, 36, 0.5, 0.5, 15, Math.toRadians(50), 1));
        aNtWoNsMoLbRaIn.add(new CurvePoint(36, 0, 0.5, 0.5, 15, Math.toRadians(50), 1));
        aNtWoNsMoLbRaIn.add(new CurvePoint(18, 36, 0.5, 0.5, 15, Math.toRadians(50), 1));
        aNtWoNsMoLbRaIn.add(new CurvePoint(0, 0, 0.5, 0.5, 15, Math.toRadians(50), 1));

        waitForStart();

        while(!isStopRequested()){
            odometry.update();
            Pose2d currentPose = odometry.getPoseEstimate();
            Point targetPoint = new Point(2,2);
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            if(drive.followCurve(aNtWoNsMoLbRaIn,270)){
                requestOpModeStop();
            }
            t.addData("Current Position (Odo): ", currentPose);
            t.update();
        }
    }
}
