package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

import java.util.ArrayList;

@Autonomous
public class LargePPPathTest extends LinearOpMode {
    SampleMecanumDrive odometry;
    RobotMovement drive;
    MultipleTelemetry t;
    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new SampleMecanumDrive(hardwareMap);
        drive = new RobotMovement(hardwareMap);
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odometry.setPoseEstimate(new Pose2d(-133, -19));

        ArrayList<CurvePoint> largePath = new ArrayList<>();
        largePath.add(new CurvePoint(-133, -19, 1, 1, 8, Math.toRadians(90), 1));
        largePath.addAll(Geometry.createArc(new Point(-44, -39), new CurvePoint(-43, -24, 1, 1, 12, Math.toRadians(90), 1), new CurvePoint(-43, -54, 1, 1, 8, Math.toRadians(90), 1), 20, true));
        largePath.addAll(Geometry.createArc(new Point(-95, -49), new CurvePoint(-96, -34, 1, 1, 10, Math.toRadians(90), 1), new CurvePoint(-96, -64, 1, 1, 8, Math.toRadians(90), 1), 20, true));
        //largePath.addAll(Geometry.createArc(new Point(-52,-60), new CurvePoint(-56, -56, 1, 1, 8, Math.toRadians(60), 1), new CurvePoint(-44, -62, 0.8, 0.8, 8, Math.toRadians(60), 1), 20, true));
        largePath.add(new CurvePoint(-52, -66, 1, 1, 6, Math.toRadians(60), 1));
        largePath.add(new CurvePoint(-44, -76, 0.4, 0.4, 3, Math.toRadians(60), 1));

        boolean purePursuit = true;

        ElapsedTime time = new ElapsedTime();

        waitForStart();
        double startTime = time.time();

        while(purePursuit && !isStopRequested()){
            odometry.update();
            Pose2d currentPose = odometry.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            if(drive.followCurve(largePath,270)) purePursuit = false;
            t.addData("Current Position (Odo): ", currentPose);
            t.update();
        }

        drive.setPower(0,0,0);
        sleep(100);
        double imuHeading = odometry.imu.getAngularOrientation().firstAngle;
        while((Math.abs(imuHeading-Math.toRadians(-90))>Math.toRadians(5)) && !isStopRequested()){
            odometry.update();
            imuHeading = odometry.imu.getAngularOrientation().firstAngle;
            int invert = (imuHeading + (360 - Math.toRadians(-90))) % 360 > 180 ? -1 : 1;
            double p = 0.3, f = 0.16;
            drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading-Math.toRadians(-90)))));
            t.update();
        }
        drive.setPower(0,0,0);

        double endTime = time.time();

        while(!isStopRequested()) {
            t.addData("Elapsed Time: ", endTime - startTime);
            t.addData("Heading: ", odometry.imu.getAngularOrientation().firstAngle);
            t.update();
        }
    }

    public void setPower(double x, double y, double a){
        drive.Motors[0].setPower(x + y + a);
        drive.Motors[1].setPower(-x + y + a);
        drive.Motors[2].setPower(x + y - a);
        drive.Motors[3].setPower(-x + y - a);
    }
}
