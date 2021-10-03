package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;

import java.util.ArrayList;

@Autonomous
public class PPTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ConfigurationRR odometry = new ConfigurationRR(hardwareMap);
        HardwareThread hardwareThread = new HardwareThread(hardwareMap, odometry);
        hardwareThread.start();
        DriveObjectRobotMovement drive = new DriveObjectRobotMovement(odometry);
        MultipleTelemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ArrayList<CurvePoint> test = new ArrayList<>();
        test.add(new CurvePoint(0, 0, 0.5, 0.5, 10, Math.toRadians(50), 1));
        test.add(new CurvePoint(0, 36, 0.5, 0.5, 10, Math.toRadians(50), 1));
        test.add(new CurvePoint(36, 36, 0.5, 0.5, 10, Math.toRadians(50), 1));
        test.add(new CurvePoint(36, 0, 0.5, 0.5, 10, Math.toRadians(50), 1));
        test.add(new CurvePoint(18, 36, 0.5, 0.5, 10, Math.toRadians(50), 1));
        test.add(new CurvePoint(0, 0, 0.5, 0.5, 10, Math.toRadians(50), 1));

        waitForStart();

        while(!isStopRequested()){
            odometry.update();
            Pose2d currentPose = odometry.getPoseEstimate();
            System.out.println("Odo pose: " + currentPose);
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            if(drive.followCurve(test,270)){
                requestOpModeStop();
            }
            odometry.update();
            currentPose = odometry.getPoseEstimate();
            t.addData("Current Position (Odo): ", currentPose);
            t.update();
        }
    }
}
