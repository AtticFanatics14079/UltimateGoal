package org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            hardware = new HardwareThread(hardwareMap, config);
            config.imu.gettingInput = true;
            waitForStart();
            ElapsedTime time = new ElapsedTime();
            hardware.start();
            while(!isStopRequested()){
                hardware.vals.waitForCycle();
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
        //double imuHeading = config.imu.get()[0];
        //double imuDegrees = Math.toDegrees(Math.abs(imuHeading));
        //imuDegrees = imuDegrees > 90 ? 180 - imuDegrees : imuDegrees;
        //imuDegrees = Math.abs(imuDegrees - 90) < 25 ? Math.abs(imuDegrees - 90) : imuDegrees;
        //double hypotenuse = config.back2.get()[0] * (1 - 0.00000706 * imuDegrees + 0.000114 * Math.pow(imuDegrees, 2));
        //telemetry.addData("Hypot: ", hypotenuse);
        //telemetry.addData("Wall: ", hypotenuse * Math.cos(Math.toRadians(imuDegrees)));
        //telemetry.addData("IMU: ", imuDegrees);
        telemetry.addData("Pose: ", sensorPoseAnalog());
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
        double headingOffsetPlus = imuHeading + Math.toRadians(12.5);
        double headingOffsetMinus = imuHeading - Math.toRadians(12.5);
        double anglePlus = Math.toDegrees(Math.abs(headingOffsetPlus)) % 90;
        anglePlus = anglePlus > 45 ? Math.abs(anglePlus - 90) : anglePlus;
        double angleMinus = Math.toDegrees(Math.abs(headingOffsetMinus)) % 90;
        angleMinus = angleMinus > 45 ? Math.abs(angleMinus - 90) : angleMinus;
        double angleCompensatorPlus = 1 - 0.0002 * anglePlus + 0.0000069 * Math.pow(anglePlus, 2) + 0.00000428 * Math.pow(anglePlus, 3);
        double angleCompensatorMinus = 1 - 0.0002 * angleMinus + 0.0000069 * Math.pow(angleMinus, 2) + 0.00000428 * Math.pow(angleMinus, 3);
        double left = config.left.get()[0];
        double right = config.right.get()[0];
        double back1 = config.back1.get()[0];
        double back2 = config.back2.get()[0];

        telemetry.addLine("Left: " + left);
        telemetry.addLine("Right: " + right);
        telemetry.addLine("Back1: " + back1);
        telemetry.addLine("Back2: " + back2);

        //Getting distance from distance sensor to either wall.
        double leftCos = left * Math.abs(Math.cos(headingOffsetPlus)) * angleCompensatorPlus;
        double leftSin = left * Math.abs(Math.sin(headingOffsetPlus)) * angleCompensatorPlus;
        double rightCos = right * Math.abs(Math.cos(headingOffsetMinus)) * angleCompensatorMinus;
        double rightSin = right * Math.abs(Math.sin(headingOffsetMinus)) * angleCompensatorMinus;
        double back1Cos = back1 * Math.abs(Math.cos(headingOffsetMinus)) * angleCompensatorMinus;
        double back1Sin = back1 * Math.abs(Math.sin(headingOffsetMinus)) * angleCompensatorMinus;
        double back2Cos = back2 * Math.abs(Math.cos(headingOffsetPlus)) * angleCompensatorPlus;
        double back2Sin = back2 * Math.abs(Math.sin(headingOffsetPlus)) * angleCompensatorPlus;

        /*telemetry.addLine("Left Cos: " + leftCos);
        telemetry.addLine("Left Sin: " + leftSin);
        telemetry.addLine("Right Cos: " + rightCos);
        telemetry.addLine("Right Sin: " + rightSin);
        telemetry.addLine("Back1 Cos: " + back1Cos);
        telemetry.addLine("Back1 Sin: " + back1Sin);
        telemetry.addLine("Back2 Cos: " + back2Cos);
        telemetry.addLine("Back2 Sin: " + back2Sin);

         */

        //Assumes radially centered.
        leftCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
        leftSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
        rightCos += sensorSideOffset * Math.abs(Math.cos(imuHeading));
        rightSin += sensorSideOffset * Math.abs(Math.sin(imuHeading));
        back1Cos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
        back1Sin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));
        back2Cos += sensorStrightOffset * Math.abs(Math.cos(imuHeading));
        back2Sin += sensorStrightOffset * Math.abs(Math.sin(imuHeading));

        /*
        telemetry.addLine("Left Cos: " + leftCos);
        telemetry.addLine("Left Sin: " + leftSin);
        telemetry.addLine("Right Cos: " + rightCos);
        telemetry.addLine("Right Sin: " + rightSin);
        telemetry.addLine("Back1 Cos: " + back1Cos);
        telemetry.addLine("Back1 Sin: " + back1Sin);
        telemetry.addLine("Back2 Cos: " + back2Cos);
        telemetry.addLine("Back2 Sin: " + back2Sin);

         */

        //Get actual X and Y of each position, assuming each input is good, based on heading for every value.
        leftCos = Math.abs(headingOffsetPlus) < Math.PI / 2 ? -leftCos : leftCos - 94; //Left or right
        leftSin = headingOffsetPlus < 0 ? -leftSin : leftSin - 142; //Front or back
        rightCos = Math.abs(headingOffsetMinus) < Math.PI / 2 ? rightCos - 94 : -rightCos; //Right or left
        rightSin = headingOffsetMinus < 0 ? rightSin - 142 : -rightSin; //Back or front
        back1Cos = Math.abs(headingOffsetMinus) < Math.PI / 2 ? back1Cos - 142 : -back1Cos; //Front or back
        back1Sin = headingOffsetMinus < 0 ? -back1Sin : back1Sin - 94; //Left or right
        back2Cos = Math.abs(headingOffsetPlus) < Math.PI / 2 ? back2Cos - 142 : -back2Cos; //Back or front
        back2Sin = headingOffsetPlus < 0 ? -back2Sin : back2Sin - 94; //Right or left

        double poseX = 0, poseY = 0;
        double confidence = 5;

        telemetry.addLine("Left Cos: " + leftCos);
        telemetry.addLine("Left Sin: " + leftSin);
        telemetry.addLine("Right Cos: " + rightCos);
        telemetry.addLine("Right Sin: " + rightSin);
        telemetry.addLine("Back1 Cos: " + back1Cos);
        telemetry.addLine("Back1 Sin: " + back1Sin);
        telemetry.addLine("Back Cos: " + back2Cos);
        telemetry.addLine("Back Sin: " + back2Sin);

        if(Math.abs(Math.cos(headingOffsetPlus)) > Math.cos(Math.toRadians(20)) || Math.abs(Math.cos(headingOffsetMinus)) > Math.cos(Math.toRadians(20))) {
            poseX = Math.abs(leftCos - rightCos) < confidence ? (leftCos + rightCos) / 2 : Math.abs(Math.cos(headingOffsetPlus)) > Math.abs(Math.cos(headingOffsetMinus)) ? leftCos : rightCos;
            poseY = Math.abs(back2Cos - back1Cos) < confidence ? (back2Cos + back1Cos) / 2 : Math.abs(Math.cos(headingOffsetPlus)) > Math.abs(Math.cos(headingOffsetMinus)) ? back2Cos : back1Cos;
        }
        else if(Math.abs(Math.sin(headingOffsetPlus)) > Math.cos(Math.toRadians(20)) || Math.abs(Math.sin(headingOffsetMinus)) > Math.cos(Math.toRadians(20))) {
            poseX = Math.abs(back2Sin - back1Sin) < confidence ? (back2Sin + back1Sin) / 2 : Math.abs(Math.sin(headingOffsetPlus)) > Math.abs(Math.sin(headingOffsetMinus)) ? back2Sin : back1Sin;
            poseY = Math.abs(leftSin - rightSin) < confidence ? (leftSin + rightSin) / 2 : Math.abs(Math.sin(headingOffsetPlus)) > Math.abs(Math.sin(headingOffsetMinus)) ? leftSin : rightSin;
        }

        //If robot sees the expected walls:
        /*double x = Math.abs(back1Cos - back2Cos) < confidence ? (back1Cos + back2Cos) / 2 : Math.abs(leftSin - rightSin) < confidence ? (leftSin + rightSin) / 2 : poseX;
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

         */

        return new Pose2d(poseX, poseY, imuHeading);
    }
}