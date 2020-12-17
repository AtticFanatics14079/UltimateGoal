package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.TeleOpMecanumDrive;

@TeleOp
public class RoadRunnerReturnToLocation extends LinearOpMode {

    private double heading = 0; // radians
    //private SampleMecanumDrive control, auton;
    private TeleOpMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        //auton = new SampleMecanumDrive(hardwareMap);
        /*DriveConstants.BASE_CONSTRAINTS = new DriveConstraints(
                90.0, 60.0, 0.0,
                Math.toRadians(180.0), Math.toRadians(180.0), 0.0
        ); //Need to modify these as well as check to make sure the changed values only impact control.
           //Trying to use DriveConstraints in lineToLinearHeading first.
         */
        //control = new SampleMecanumDrive(hardwareMap);
        drive = new TeleOpMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;
        while (!isStopRequested()) {
            takeInput();
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    public void takeInput() {
        if(gamepad1.x) {
            drive.setPoseEstimate(new Pose2d(0, 0));
            heading = drive.getRawExternalHeading();
        }
        if(gamepad1.y) {
            for(DcMotor d : drive.motors) d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.setPoseEstimate(drive.getPoseEstimate());
            Trajectory goToShoot = drive.trajectoryBuilder(drive.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(0, 0, heading), new DriveConstraints(
                            40.0, 20.0, 0.0,
                            Math.toRadians(180.0), Math.toRadians(180.0), 0.0))
                    .build();
            drive.followTrajectory(goToShoot);
            System.out.println("Here");
            DriveConstants.BASE_CONSTRAINTS = new DriveConstraints(80.0, 60.0,
                    0.0, Math.toRadians(180.0), Math.toRadians(180.0), 0.0);
        }
        else {
            for(DcMotor d : drive.motors) d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void setPower(double px, double py, double pa){ //Multiplied pa by -1 to suit turning requests
        double p1 = -px + py + pa;
        double p2 = px + py + pa;
        double p3 = -px + py - pa;
        double p4 = px + py - pa;
        double max = Math.max(1.0, Math.abs(p1));
        max = Math.max(max, Math.abs(p2));
        max = Math.max(max, Math.abs(p3));
        max = Math.max(max, Math.abs(p4));
        p1 /= max;
        p2 /= max;
        p3 /= max;
        p4 /= max;
        drive.motors.get(0).setPower(p1);
        drive.motors.get(1).setPower(p2);
        drive.motors.get(2).setPower(p3);
        drive.motors.get(3).setPower(p4);
    }
}
