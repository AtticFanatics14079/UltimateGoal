package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareConfigs.OneHubDrive;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.TeleOp.UltimateGoalTeleOpV1.multiplier;
import static org.firstinspires.ftc.teamcode.TeleOp.UltimateGoalTeleOpV1.sensorSideOffset;
import static org.firstinspires.ftc.teamcode.TeleOp.UltimateGoalTeleOpV1.sensorStrightOffset;

@TeleOp
public class SampleTeleOp extends LinearOpMode {

    HardwareThread hardware;
    SampleConfiguration config;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            config = new SampleConfiguration();
            ValueStorage vals = new ValueStorage();
            hardware = new HardwareThread(hardwareMap, vals, config);
            //hardware.config.ExtendGripper.setPID(2, 0, 0); //Gonna need to mess with this one
            config.imu.gettingInput = true;
            waitForStart();
            ElapsedTime time = new ElapsedTime();
            hardware.start();
            while(!isStopRequested()){
                vals.waitForCycle();
                System.out.println("Finshed waiting, " + time.milliseconds());
                getInput();
            }
        } catch(Exception e) {
            System.out.println("Exception: " + e);
        } finally {
            hardware.Stop();
        }
    }

    private void getInput(){
        setPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        double imuHeading = config.imu.get()[0];
        double imuDegrees = Math.toDegrees(imuHeading);
        double hypotenuse = config.back2.get()[0] * (1 - 0.00000706 * imuDegrees + 0.000114 * Math.pow(imuDegrees, 2));
        telemetry.addData("Hypot: ", hypotenuse);
        telemetry.addData("Adjacent: ", hypotenuse * Math.cos(imuHeading));
        telemetry.addData("IMU: ", Math.toDegrees(imuHeading));
        telemetry.update();
    }

    private void setPower(double px, double py, double pa){
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
        config.backLeft.setPower(p1);
        config.frontLeft.setPower(p2);
        config.frontRight.setPower(p3);
        config.backRight.setPower(p4);
    }

    public Pose2d sensorPoseAnalog() {
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        //CURRENTLY THE SYSTEM PERPENDICULAR TO WALLS since I would like to test whether the system works before I do fancy stuff.

        //As of April 6, the sensors appear to have a tolerance for 40 degrees either side, with distance not seeming to matter. I would go 30 degrees to be safe.

        System.out.println("1");

        //config.imu.gettingInput = true;
        config.imu.pingSensor();
        config.vals.waitForCycle();
        if(isStopRequested()) return null;
        double imuHeading = config.imu.get()[0];
        double angleCompensator = 1 - 0.0002 * imuHeading + 0.0000069 * Math.pow(imuHeading, 2) + 0.00000428 * Math.pow(imuHeading, 3);
        double left = config.left.get()[0] * angleCompensator;
        double right = config.right.get()[0] * angleCompensator;
        double back1 = config.back1.get()[0] * angleCompensator;
        double back2 = config.back2.get()[0] * angleCompensator;

        double headingOffsetPlus = Math.toRadians(20) + imuHeading;
        double headingOffsetMinus = Math.toRadians(20) - imuHeading;

        //Getting distance from distance sensor to either wall.
        double leftCos = left * Math.abs(Math.cos(headingOffsetPlus));
        double leftSin = left * Math.abs(Math.sin(headingOffsetPlus));
        double rightCos = right * Math.abs(Math.cos(headingOffsetMinus));
        double rightSin = right * Math.abs(Math.sin(headingOffsetMinus));
        double back1Cos = back1 * Math.abs(Math.cos(headingOffsetMinus));
        double back1Sin = back1 * Math.abs(Math.sin(headingOffsetMinus));
        double back2Cos = back2 * Math.abs(Math.cos(headingOffsetPlus));
        double back2Sin = back2 * Math.abs(Math.sin(headingOffsetPlus));

        //Assumes radially centered.
        leftCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
        leftSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
        rightCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
        rightSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
        back1Cos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
        back1Sin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));
        back2Cos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
        back2Sin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));

        //Get actual X and Y of each position, assuming each input is good, based on heading for every value.
        leftCos = Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0 ? 10 : Math.abs(headingOffsetPlus) < Math.PI / 2 ? 9 - leftCos : leftCos - 87; //Left or right
        leftSin = Math.abs(Math.sin(headingOffsetPlus)) < Math.sqrt(3)/2.0 ? 10 : headingOffsetPlus < 0 ? 9 - leftSin : leftSin - 135; //Front or back
        rightCos = Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0 ? 10 : Math.abs(headingOffsetMinus) < Math.PI / 2 ? rightCos - 87 : 9 - rightCos; //Right or left
        rightSin = Math.abs(Math.sin(headingOffsetMinus)) < Math.sqrt(3)/2.0 ? 10 : headingOffsetMinus < 0 ? rightSin - 135 : 9 - rightSin; //Back or front
        back1Cos = Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0 ? 10 : Math.abs(headingOffsetMinus) < Math.PI / 2 ? 9 - back1Cos : back1Cos - 135; //Front or back
        back1Sin = Math.abs(Math.sin(headingOffsetMinus)) < Math.sqrt(3)/2.0 ? 10 : headingOffsetMinus < 0 ? back1Sin - 87 : 9 - back1Sin; //Left or right
        back2Cos = Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0 ? 10 : Math.abs(headingOffsetPlus) < Math.PI / 2 ? back2Cos - 135 : 9 - back2Cos; //Back or front
        back2Sin = Math.abs(Math.sin(headingOffsetPlus)) < Math.sqrt(3)/2.0 ? 10 : headingOffsetPlus < 0 ? 9 - back2Sin : back2Sin - 87; //Right or left

        double poseX = 0, poseY = 0;
        double confidence = 5;

        telemetry.addLine("Left Cos: " + leftCos);
        telemetry.addLine("Left Sin: " + leftSin);
        telemetry.addLine("Right Cos: " + rightCos);
        telemetry.addLine("Right Sin: " + rightSin);
        telemetry.addLine("Front Cos: " + back1Cos);
        telemetry.addLine("Front Sin: " + back1Sin);
        telemetry.addLine("Back Cos: " + back2Cos);
        telemetry.addLine("Back Sin: " + back2Sin);

        //If robot sees the expected walls:
        double x = Math.abs(back1Cos - back2Cos) < confidence ? (back1Cos + back2Cos) / 2 : Math.abs(leftSin - rightSin) < confidence ? (leftSin + rightSin) / 2 : poseX;
        double y = Math.abs(leftCos - rightCos) < confidence ? (leftCos + rightCos) / 2 : Math.abs(back1Sin - back2Sin) < confidence ? (back1Sin + back2Sin) / 2 : poseY;

        if(x == poseX) {
            //Don't have both inputs, check for angle then choose from viable options.
            if(Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0 && Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0) x = Math.min(back1Cos, back2Cos); //Would compare to odo
            else if(Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0) x = back2Cos;
            else if(Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0) x = back1Cos;
            else if(Math.abs(Math.sin(headingOffsetPlus)) < Math.sqrt(3)/2.0 && Math.abs(Math.sin(headingOffsetMinus)) < Math.sqrt(3)/2.0) x = Math.min(leftSin, rightSin); //Would compare to odo
            else if(Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0) x = leftSin;
            else if(Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0) x = rightSin;
        }
        if(y == poseY) {
            //Don't have both inputs, check for angle then choose from viable options.
            if(Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0 && Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0) y = Math.min(leftCos, rightCos); //Would compare to odo
            else if(Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0) y = leftCos;
            else if(Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0) y = rightCos;
            else if(Math.abs(Math.sin(headingOffsetPlus)) < Math.sqrt(3)/2.0 && Math.abs(Math.sin(headingOffsetMinus)) < Math.sqrt(3)/2.0) y = Math.min(back1Sin, back2Sin); //Would compare to odo
            else if(Math.abs(Math.cos(headingOffsetPlus)) < Math.sqrt(3)/2.0) y = back2Sin;
            else if(Math.abs(Math.cos(headingOffsetMinus)) < Math.sqrt(3)/2.0) y = back1Sin;
        }

        System.out.println("PoseX: " + x + ", Pose Y: " + y);

        return new Pose2d(x, y, imuHeading);
    }
}