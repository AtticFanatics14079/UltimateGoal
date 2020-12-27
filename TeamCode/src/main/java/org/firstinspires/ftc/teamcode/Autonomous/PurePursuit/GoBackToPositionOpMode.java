package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;

@TeleOp
public class GoBackToPositionOpMode extends LinearOpMode {
    SampleMecanumDrive odometry;
    RobotMovement drive;
    MultipleTelemetry t;
    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new SampleMecanumDrive(hardwareMap);
        drive = new RobotMovement(hardwareMap);
        t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(!isStopRequested()){
            odometry.update();
            Pose2d currentPose = odometry.getPoseEstimate();
            Pose2d targetPose = new Pose2d(0,0, Math.toRadians(0));
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            setPower(0*-gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            if(gamepad1.start){
                while((Math.hypot(Math.abs(targetPose.getX()-currentPose.getX()),Math.abs(targetPose.getY()-currentPose.getY()))>0.5)&&!isStopRequested()){
                    odometry.update();
                    currentPose = odometry.getPoseEstimate();
                    drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
                    t.addData("Current Position (Odo): ", currentPose);
                    t.addData("Target Point: ", targetPose);
                    t.update();
                    drive.goToPosition(targetPose.getX(),targetPose.getY(),0.7,90,0.3);
                }
                drive.setPower(0,0,0);
                sleep(500);
                double imuHeading = odometry.imu.getAngularOrientation().firstAngle;
                while((Math.abs(imuHeading-targetPose.getHeading())>Math.toRadians(2)) && !isStopRequested()){
                    odometry.update();
                    imuHeading = odometry.imu.getAngularOrientation().firstAngle;
                    currentPose = odometry.getPoseEstimate();
                    int invert = (imuHeading + (360 - targetPose.getHeading())) % 360 > 180 ? -1 : 1;
                    telemetry.addData("Bic Invert: ", invert);
                    double p = 0.2, f = 0.15;
                    drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading-targetPose.getHeading()))));
                    t.addData("Current Position (Odo): ", currentPose);
                    t.addData("Target Point: ", targetPose);
                    t.update();
                }
                odometry.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
            }
            else if(gamepad1.x){
                odometry.setPoseEstimate((new Pose2d(0,0,odometry.imu.getAngularOrientation().firstAngle)));
            }
            t.addData("Current Position (Odo): ", currentPose);
            t.addData("Target Point: ", targetPose);
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
