package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.HardwareConfigs.LimitConfiguration;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {

    SampleMecanumDrive config;

    @Override
    public void runOpMode() throws InterruptedException {

        config  = new SampleMecanumDrive(hardwareMap);

        DistanceSensor leSense = hardwareMap.get(DistanceSensor.class, "distanceRight");

        config.setPoseEstimate(new Pose2d(0, 0, 0));

        while(!isStopRequested()) {

            config.update();

            setPower(-gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x);

            telemetry.addData("Pose: ", config.getPoseEstimate());
            telemetry.update();

            if(gamepad1.start) {
                Pose2d currentPose = config.getPoseEstimate();
                double imuHeading = config.imu.getAngularOrientation().firstAngle;
                while((Math.abs(imuHeading)>Math.toRadians(2)) && !isStopRequested()){
                    config.update();
                    imuHeading = config.imu.getAngularOrientation().firstAngle;
                    currentPose = config.getPoseEstimate();
                    double p = 0.2, f = 0.15;
                    setPower(0,0, (f+(p*imuHeading)));
                }
                config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));

                telemetry.addData("Distance (in): ", leSense.getDistance(DistanceUnit.INCH));
                telemetry.addData("Odo Pose: ", config.getPoseEstimate());
                telemetry.update();

                sleep(5000);
            }
        }
    }

    public void setPower(double x, double y, double a){
        config.motors.get(0).setPower(x + y + a);
        config.motors.get(1).setPower(-x + y + a);
        config.motors.get(2).setPower(x + y - a);
        config.motors.get(3).setPower(-x + y - a);
    }
}