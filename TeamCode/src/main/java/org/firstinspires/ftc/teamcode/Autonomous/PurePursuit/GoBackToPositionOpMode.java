package org.firstinspires.ftc.teamcode.Autonomous.PurePursuit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            Point targetPoint = new Point(0,0);
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            setPower(0,-gamepad1.left_stick_y,gamepad1.right_stick_x);
            if(gamepad1.start){
                while((Math.hypot(Math.abs(targetPoint.x-currentPose.getX()),Math.abs(targetPoint.y-currentPose.getY()))>1.0)&&!isStopRequested()){
                    odometry.update();
                    currentPose = odometry.getPoseEstimate();
                    drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
                    t.addData("Current Position (Odo): ", currentPose);
                    t.addData("Target Point: ", targetPoint);
                    t.update();
                    drive.goToPosition(targetPoint.x,targetPoint.y,0.3,270,1);
                }
            }
            t.addData("Current Position (Odo): ", currentPose);
            t.addData("Target Point: ", targetPoint);
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
