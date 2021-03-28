package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.DriveObjectRobotMovement;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.Point;
import org.firstinspires.ftc.teamcode.Autonomous.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.HardwareConfigs.UltimateGoalConfig;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Configuration;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ConfigurationRR;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DEncoderlessMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DMotor;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.DThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.HardwareThread;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.Sequence;
import org.firstinspires.ftc.teamcode.JONSKETCH.DriveObjectV2.ValueStorage;
import org.firstinspires.ftc.teamcode.Vision.twoScanPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp
public class UltimateGoalTeleOpV1 extends LinearOpMode {

    private final int rows = 640;
    private final int cols = 480;
    public static int sampleWidth = 30;
    public static int sampleHeight = 3, leftOff = 20;
    public static org.opencv.core.Point topCenter = new org.opencv.core.Point(510, 420);
    public static org.opencv.core.Point bottomCenter = new org.opencv.core.Point(510, 350);
    public static org.opencv.core.Point leftBar1 = new org.opencv.core.Point(442, 360), leftBar2 = new org.opencv.core.Point(451, 436), rightBar1 = new org.opencv.core.Point(198, 359), rightBar2 = new org.opencv.core.Point(207, 435);
    public static int thresh = 140, redThresh = 137;
    public static int wobbleThresh = 145, initThresh = 133, targetHighGoalX = 180, pshotLeft = 64, pshotMid = 64, pshotRight = 64; //Right does not see far left powershot, calibrate these with Dash
    public static int stackSize = -1;
    private static double color1, color2;
    public static boolean initDetect = true, lameMode = true;
    public static boolean properSetup = false;
    public static double offsetDivisor = 50;
    public static double rotateAngle = 180;

    public static double fastImuP = 0.5, fastImuF = 0.15, highGoalFastP = 0.0005, highGoalFastF = 0.08;

    public static int upperCameraCenter = 0;
    public static double maxX = -1;
    public static int height = 130;

    public static int extract = 1;
    public static int row = 320;

    OpenCvCamera webCam, webcam2;


    ConfigurationRR config;
    DriveObjectRobotMovement drive;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;
    Thread returnToShoot, powerShots, shootMacro, grabWobble, dropWobble, lowerWobble;
    public static double wobbleUp = 0.4, wobbleDown = 0.85, wobbleMid = 0.67, gripperOpen = 0, gripperClosed = 1, load = 0.5, reload = 0.1, shooterSpeed = -1640, multiplier = 1, sensorSideOffset = 8.20, sensorSideAngle = 0.66, sensorStrightOffset = 9, sensorStraightAngle = 0, rightDistMult = 1; //Sheets had 1.15 as multiplier, seeing if just my house that's off

    public static double highGoalX = 0, highGoalY = 0, powerShotX = 0, powerShotY = 0, wallDistance = 18, distanceLeft = 21, distanceRight = 15;

    Pose2d highGoalShoot = new Pose2d(highGoalX, highGoalY, 0);
    Pose2d powerShotShoot = new Pose2d(powerShotX, powerShotY, 0);

    DistanceSensor leSense;

    ElapsedTime time;

    private boolean lockedLoader = false, pressedLock = false, pressedShooter = false, shooterFast = false, pressedOdoAdjust = false;

    double intakeSpeed = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            config = new ConfigurationRR(hardwareMap);
            hardware = new HardwareThread(hardwareMap, vals, config);
            hardware.start();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), cameraMonitorViewId);
            webcam2.openCameraDevice();//open camera
            webcam2.setPipeline(new upperCameraPipeline());//different stages
            webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC
            drive = new DriveObjectRobotMovement(config);
            for(DEncoderlessMotor d : config.motors) d.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            configureMacros();
            config.setPoseEstimate(new Pose2d(0, 0, 0));
            waitForStart();
            time = new ElapsedTime();
            if(!isStopRequested()) {
                //config.setPoseEstimate(new Pose2d(-1, 0, 0));
                while(opModeIsActive()){
                    vals.waitForCycle();
                    getInput();
                }
            }
        } catch(Exception e){
            System.out.println(e);
        } finally{
            hardware.Stop();
        }

    }

    public void getInput(){
        //Main loop
        System.out.println("Input loop: " + time.milliseconds());
        config.update();
        //config.imu.retrievingHardware(true);
        config.setPoseEstimate(sensorPoseNew());
        telemetry.addData("Pose: ", config.getPoseEstimate());
        //double head = config.imu.get()[0];
        telemetry.update();
        if(gamepad2.a && !pressedShooter) {
            pressedShooter = true;
            shooterFast = !shooterFast;
        }
        else if(pressedShooter && !gamepad2.a) pressedShooter = false;
        if(gamepad1.right_stick_button) {
            config.setPoseEstimate(sensorPose());
            Pose2d currentPose = config.getPoseEstimate();
            System.out.println("Current pose: " + currentPose);
            System.out.println("Angle: " + Math.atan((currentPose.getY() + 64) / currentPose.getX()));
            double dist = Math.sqrt(Math.pow((currentPose.getY() + 60), 2) + Math.pow(currentPose.getX(), 2));
            System.out.println("Distance: " + dist);
            if(dist < 96) shooterSpeed = -1600;
            else if(dist < 105) shooterSpeed = -1620;
            else if(dist < 112) shooterSpeed = -1640;
            else shooterSpeed = -1660;
            telemetry.addData("Current Pose: ", currentPose);
            telemetry.addData("Angle: ", Math.atan((currentPose.getY() + 64) / currentPose.getX()));
            telemetry.update();
            imuTurn(Math.atan((currentPose.getY() + 64) / currentPose.getX())); //Testing 60, there was a consistent trend of turning too far left. May need to later either change the target or add a constant turn to better match camera.
            sleep(300);
            turnUntilHighGoal((int)(100 / dist * 30));
        }
        if(gamepad1.left_stick_button) {
            config.setPoseEstimate(sensorPoseNew());
            Pose2d currentPose = config.getPoseEstimate();
            System.out.println("Current pose: " + currentPose);
            System.out.println("Angle: " + Math.atan((currentPose.getY() + 64) / currentPose.getX()));
            double dist = Math.sqrt(Math.pow((currentPose.getY() + 60), 2) + Math.pow(currentPose.getX(), 2));
            System.out.println("Distance: " + dist);
            if(dist < 96) shooterSpeed = -1600;
            else if(dist < 105) shooterSpeed = -1620;
            else if(dist < 112) shooterSpeed = -1640;
            else shooterSpeed = -1660;
            telemetry.addData("Current Pose: ", currentPose);
            telemetry.addData("Angle: ", Math.atan((currentPose.getY() + 64) / currentPose.getX()));
            telemetry.update();
            imuTurnFast(Math.atan((currentPose.getY() + 64) / currentPose.getX())); //Testing 60, there was a consistent trend of turning too far left. May need to later either change the target or add a constant turn to better match camera.
            sleep(200);
            turnUntilHighGoalFast((int)(100 / dist * 30));
        }
        if(gamepad2.start) {
            config.setPoseEstimate(new Pose2d(0, 0, 0));
            config.imu.resetIMU();
        }
        if(gamepad2.left_stick_y > 0.5) shooterSpeed = -1540;
        else if(gamepad2.left_stick_y < -0.5) shooterSpeed = -1640;
        if(!shootMacro.isAlive()) config.shooter.set(shooterFast ? shooterSpeed : 0);
        if(gamepad1.start && !shootMacro.isAlive()) {
            returnToShoot.start();
            while(returnToShoot.isAlive());
        }
        else if(gamepad1.a && !shootMacro.isAlive() && config.ingester.get()[0] >= shooterSpeed - 500) { //Just making sure we don't shoot until after shooter reaches speed, may add this check to the macro.
            shootMacro.start();
        }
        else if(gamepad1.back) {
            powerShots.start();
            while(powerShots.isAlive());
        }
        System.out.println("Time 1: " + time.milliseconds());
        double speedMultiplier = -1;
        if(gamepad1.left_bumper) speedMultiplier = -0.3;
        //if(gamepad1.right_bumper) setPower(speedMultiplier * (gamepad1.left_stick_x * Math.cos(head) + gamepad1.left_stick_y * Math.sin(head)), speedMultiplier * (gamepad1.left_stick_x * Math.sin(head) + gamepad1.left_stick_y * Math.cos(head)), -speedMultiplier * gamepad1.right_stick_x); //Should be field-centric
        setPower(speedMultiplier * gamepad1.left_stick_x, speedMultiplier * gamepad1.left_stick_y, speedMultiplier * gamepad1.right_stick_x);
        if(shootMacro.isAlive()) {}
        else if((gamepad1.b || gamepad2.back) && config.ingester.get()[0] >= shooterSpeed - 500) config.loader.set(load);
        else config.loader.set(reload);

        if(gamepad1.x && !grabWobble.isAlive() && !dropWobble.isAlive() && !lowerWobble.isAlive()) grabWobble.start();
        else if(gamepad2.x && !grabWobble.isAlive() && !dropWobble.isAlive() && !lowerWobble.isAlive()) dropWobble.start();
        else if((gamepad2.y || gamepad1.y) && !grabWobble.isAlive() && !dropWobble.isAlive() && !lowerWobble.isAlive()) lowerWobble.start();

        if(grabWobble.isAlive() || dropWobble.isAlive()) {}
        else if(gamepad2.left_bumper && !grabWobble.isAlive()) config.gripper.set(gripperClosed); //Change to g2
        else if(gamepad2.right_bumper) config.gripper.set(gripperOpen); //Change to g2

        if(grabWobble.isAlive() || dropWobble.isAlive() || lowerWobble.isAlive()) {}
        else if(gamepad2.left_trigger > 0.2) config.wobble.set(wobbleUp); //Change to g2
        else if(gamepad2.right_trigger > 0.2) config.wobble.set(wobbleDown); //Change to g2

        if(gamepad2.dpad_up || gamepad1.dpad_right) intakeSpeed = -1;
        else if(gamepad2.b) intakeSpeed = 0;
        else if(gamepad2.dpad_down || gamepad1.dpad_left) intakeSpeed = 1;
        config.ingester.set(intakeSpeed);
        config.preIngest.set(-intakeSpeed);
    }

    public void configureMacros() {
        Sequence returnToHighGoalDistance = new Sequence(() -> {
            config.setPoseEstimate(sensorPose());
            System.out.println("Pose: " + config.getPoseEstimate());
            config.shooter.set(-1640);
            roadRunnerToPosition(new Pose2d(-90, -62));
            imuTurn(0);
            config.setPoseEstimate(sensorPose());
            turnUntilHighGoal(30);
            return null;
        });
        Sequence tripleShoot = new Sequence(() -> {
            config.shooter.set(shooterSpeed);
            shootOnce();
            for(int i = 0; i < 2; i++) {
                sleep(400);
                shootOnce();
            }
            return null;
        }, returnToHighGoalDistance);
        returnToShoot = new Thread(tripleShoot);

        Sequence shootThrice = new Sequence(() -> {
            config.shooter.set(shooterSpeed + 20);
            for(int i = 0; i < 2; i++) {
                sleep(400);
                shootOnce();
                config.shooter.set(shooterSpeed);
            }
            config.shooter.set(shooterSpeed - 20);
            sleep(400);
            shootOnce();
            return null;
        });
        shootMacro = new Thread(shootThrice);

        Sequence returnToPowerShot = new Sequence(() -> {
            config.setPoseEstimate(sensorPose());
            config.shooter.set(-1520);
            roadRunnerToPositionSlow(new Pose2d(-80, -30));
            config.setPoseEstimate(sensorPose());
            imuTurn(0);
            sleep(400);
            turnToPowershot(pshotRight);
            return null;
        });
        Sequence pivotShoot = new Sequence(() -> {
            sleep(400);
            shootOnce();
            imuTurn(0);
            sleep(400);
            turnToPowershot(pshotMid);
            sleep(400);
            shootOnce();
            sleep(400);
            turnToPowershot(pshotLeft);
            sleep(400);
            shootOnce();
            config.loader.set(load);
            return null;
        }, returnToPowerShot);
        powerShots = new Thread(pivotShoot);

        Sequence grab = new Sequence(() -> {
            config.gripper.set(gripperClosed);
            sleep(1200);
            config.wobble.set(wobbleUp);
            return null;
        });
        grabWobble = new Thread(grab);

        Sequence drop = new Sequence(() -> {
            config.wobble.set(wobbleMid);
            sleep(200);
            config.gripper.set(gripperOpen);
            return null;
        });
        dropWobble = new Thread(drop);

        Sequence lowerToGrab = new Sequence(() -> {
            config.wobble.set(wobbleDown);
            sleep(400);
            config.gripper.set(gripperOpen);
            return null;
        });
        lowerWobble = new Thread(lowerToGrab);
    }

    /*public void imuTurn() {
        config.imu.retrievingHardware(true);
        sleep(40);
        double imuHeading = config.imu.get()[0];
        drive.runWithEncoder(true);
        Pose2d currentPose = config.getPoseEstimate();
        while((Math.abs(imuHeading)>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
            config.update();
            imuHeading = config.imu.get()[0];
            if(imuHeading < 0) imuHeading += 2 * Math.PI;
            currentPose = config.getPoseEstimate();
            double p = 0.35, f = 0.04;
            int invert = ((2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double power = invert * p * (Math.abs(imuHeading) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((0 > Math.PI ? 2 * Math.PI : 0) - 0)) : Math.abs(imuHeading - 0)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
        }
        drive.setPower(0, 0, 0);
        config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
        config.imu.retrievingHardware(false);
    }
     */

    public void imuTurn(double angle) {
        //Radians
        config.imu.retrievingHardware(true);
        vals.waitForCycle();
        double imuHeading = config.imu.get()[0];
        drive.runWithEncoder(true);
        while((Math.abs(imuHeading - angle)>Math.toRadians(5)) && !isStopRequested() && opModeIsActive()){
            System.out.println("Heading check: " + Math.abs(imuHeading - angle));
            System.out.println("Angle: " + angle);
            config.update();
            imuHeading = config.imu.get()[0];
            double tempHeading = imuHeading;
            double tempTarget = angle;
            System.out.println("Heading: " + imuHeading);
            if(tempHeading < 0) tempHeading += 2 * Math.PI;
            if(angle < 0) tempTarget += 2 * Math.PI;
            double p = 0.35, f = 0.04;
            //int invert = tempHeading + (2 * Math.PI - tempTarget) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double invert = angle - imuHeading;
            if(invert > Math.PI) invert -= 2 * Math.PI;
            else if(invert < -Math.PI) invert += 2 * Math.PI;
            invert = invert < 0 ? 1 : -1;
            double power = invert * p * (Math.abs(tempHeading - tempTarget) > Math.PI ? (Math.abs(tempHeading > Math.PI ? 2 * Math.PI - tempHeading : tempHeading) + Math.abs(tempTarget > Math.PI ? 2 * Math.PI - tempTarget : tempTarget)) : Math.abs(tempHeading - tempTarget)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
        }
        drive.setPower(0, 0, 0);
        config.imu.retrievingHardware(false);
    }

    public void imuTurnFast(double angle) {
        //Radians
        config.imu.retrievingHardware(true);
        vals.waitForCycle();
        vals.waitForCycle();
        config.imu.retrievingHardware(true);
        double imuHeading = config.imu.get()[0];
        drive.runWithEncoder(true);
        while((Math.abs(imuHeading - angle)>Math.toRadians(10)) && !isStopRequested() && opModeIsActive()){
            System.out.println("Heading check: " + Math.abs(imuHeading - angle));
            System.out.println("Angle: " + angle);
            config.update();
            imuHeading = config.imu.get()[0];
            double tempHeading = imuHeading;
            double tempTarget = angle;
            System.out.println("Heading: " + imuHeading);
            if(tempHeading < 0) tempHeading += 2 * Math.PI;
            if(angle < 0) tempTarget += 2 * Math.PI;
            double p = fastImuP, f = fastImuF;
            //int invert = tempHeading + (2 * Math.PI - tempTarget) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double invert = angle - imuHeading;
            if(invert > Math.PI) invert -= 2 * Math.PI;
            else if(invert < -Math.PI) invert += 2 * Math.PI;
            invert = invert < 0 ? 1 : -1;
            double power = invert * p * (Math.abs(tempHeading - tempTarget) > Math.PI ? (Math.abs(tempHeading > Math.PI ? 2 * Math.PI - tempHeading : tempHeading) + Math.abs(tempTarget > Math.PI ? 2 * Math.PI - tempTarget : tempTarget)) : Math.abs(tempHeading - tempTarget)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
            vals.waitForCycle();
            System.out.println("End of cycle");
        }
        drive.setPower(0, 0, 0);
        config.imu.retrievingHardware(false);
    }

    public void roadRunnerToPosition(Pose2d targetPose) {
        drive.runWithEncoder(true);
        Pose2d currentPose = config.getPoseEstimate();
        double halfX = (currentPose.getX() + targetPose.getX()) / 2;
        double halfY = (currentPose.getY() + targetPose.getY()) / 2;
        Trajectory traj = config.trajectoryBuilder(currentPose)
                .strafeTo(targetPose.vec())
                .build();
        config.followTrajectory(traj);
        drive.setPower(0,0,0);
        drive.runWithEncoder(false);
    }

    public void roadRunnerToPositionSlow(Pose2d targetPose) {
        drive.runWithEncoder(true);
        Pose2d currentPose = config.getPoseEstimate();
        Trajectory traj = config.trajectoryBuilder(currentPose)
                .strafeTo(targetPose.vec(), new DriveConstraints(
                    30.0, 20.0, 0.0,
                    Math.toRadians(120.0), Math.toRadians(120.0), 0.0
                ))
                .build();
        config.followTrajectory(traj);
        drive.setPower(0,0,0);
        drive.runWithEncoder(false);
    }

    public Pose2d sensorPoseNew() {
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        System.out.println("1");

        //config.imu.gettingInput = true;
        config.rightDist.pingSensor();
        config.leftDist.pingSensor();
        config.frontDist.pingSensor();
        config.backDist.pingSensor();
        config.imu.pingSensor();
        vals.waitForCycle();
        if(isStopRequested()) return null;
        double imuHeading = config.imu.get()[0];
        double left = config.leftDist.getDistance(DistanceUnit.INCH);
        double right = config.rightDist.getDistance(DistanceUnit.INCH);
        double front = config.frontDist.getDistance(DistanceUnit.INCH);
        double back = config.backDist.getDistance(DistanceUnit.INCH);
        double correctedHeading = imuHeading % Math.PI; //Correct heading in each quadrant, as a new quadrant switches what wall it should be seeing.

        System.out.println("2");
        System.out.println("Left: " + left);
        System.out.println("Right: " + right);
        System.out.println("Front: " + front);
        System.out.println("Back: " + back);

        //Getting distance from distance sensor to either wall.
        double leftCos = left * Math.abs(Math.cos(correctedHeading)) * multiplier;
        double leftSin = left * Math.abs(Math.sin(correctedHeading)) * multiplier;
        double rightCos = right * Math.abs(Math.cos(correctedHeading)) * multiplier;
        double rightSin = right * Math.abs(Math.sin(correctedHeading)) * multiplier;
        double frontCos = front * Math.abs(Math.cos(correctedHeading)) * multiplier;
        double frontSin = front * Math.abs(Math.sin(correctedHeading)) * multiplier;
        double backCos = back * Math.abs(Math.cos(correctedHeading)) * multiplier;
        double backSin = back * Math.abs(Math.sin(correctedHeading)) * multiplier;

        System.out.println("3");
        System.out.println("Left Cos: " + leftCos);
        System.out.println("Left Sin: " + leftSin);
        System.out.println("Right Cos: " + rightCos);
        System.out.println("Right Sin: " + rightSin);
        System.out.println("Front Cos: " + frontCos);
        System.out.println("Front Sin: " + frontSin);
        System.out.println("Back Cos: " + backCos);
        System.out.println("Back Sin: " + backSin);
        System.out.println();

        //Converting distance from sensors to distance from robot center.
        /*leftCos += Math.abs(Math.cos(sensorSideAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));
        leftSin += Math.abs(Math.sin(sensorSideAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));
        rightCos += Math.abs(Math.cos(-sensorSideAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading))); //Test using negative sensorSideAngle more, I think it's wrong.
        rightSin += Math.abs(Math.sin(-sensorSideAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));
        frontCos += Math.abs(Math.cos(sensorStraightAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));
        frontSin += Math.abs(Math.sin(sensorStraightAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));
        backCos += Math.abs(Math.cos(-sensorStraightAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));
        backSin += Math.abs(Math.sin(-sensorStraightAngle + Math.max(correctedHeading, Math.PI / 2 - correctedHeading)));\
         */

        //double sensorAngle = Math.tan(imuHeading) < 0 ? Math.PI / 2 - correctedHeading : correctedHeading;
        leftCos += sensorSideOffset * Math.abs(Math.cos(correctedHeading));
        leftSin += sensorSideOffset * Math.abs(Math.sin(correctedHeading));
        rightCos += sensorSideOffset * Math.abs(Math.cos(correctedHeading));
        rightSin += sensorSideOffset * Math.abs(Math.sin(correctedHeading));
        frontCos += sensorStrightOffset * Math.abs(Math.cos(correctedHeading));
        frontSin += sensorStrightOffset * Math.abs(Math.sin(correctedHeading));
        backCos += sensorStrightOffset * Math.abs(Math.cos(correctedHeading));
        backSin += sensorStrightOffset * Math.abs(Math.sin(correctedHeading));

        System.out.println("4");

        System.out.println("Left Cos: " + leftCos);
        System.out.println("Left Sin: " + leftSin);
        System.out.println("Right Cos: " + rightCos);
        System.out.println("Right Sin: " + rightSin);
        System.out.println("Front Cos: " + frontCos);
        System.out.println("Front Sin: " + frontSin);
        System.out.println("Back Cos: " + backCos);
        System.out.println("Back Sin: " + backSin);
        System.out.println();

        //Get actual X and Y of each position, assuming each input is good, based on heading for every value.
        leftCos = Math.abs(imuHeading) < Math.PI / 2 ? 9 - leftCos : leftCos - 87; //Left or right
        leftSin = imuHeading < 0 ? 9 - leftSin : leftSin - 135; //Front or back
        rightCos = Math.abs(imuHeading) < Math.PI / 2 ? rightCos - 87 : 9 - rightCos; //Right or left
        rightSin = imuHeading < 0 ? rightSin - 135 : 9 - rightSin; //Back or front
        frontCos = Math.abs(imuHeading) < Math.PI / 2 ? 9 - frontCos : frontCos - 135; //Front or back
        frontSin = imuHeading < 0 ? frontSin - 87 : 9 - frontSin; //Left or right
        backCos = Math.abs(imuHeading) < Math.PI / 2 ? backCos - 135 : 9 - backCos; //Back or front
        backSin = imuHeading < 0 ? 9 - backSin : backSin - 87; //Right or left

        //TODO: Do comparisons to determine which wall each distance sensor is seeing. Do this by first checking to see if opposites actually see opposites for cos and sin.

        //MARCH 20 NOTES: Appears to be working fine for cases that worked previously, but needs more testing on off cases to find what is wrong (and it is). Changed the way of finding distance to center.

        Pose2d currentPose = config.getPoseEstimate();
        double poseX = currentPose.getX(), poseY = currentPose.getY();
        double confidence = 4;

        System.out.println("Left Cos: " + leftCos);
        System.out.println("Left Sin: " + leftSin);
        System.out.println("Right Cos: " + rightCos);
        System.out.println("Right Sin: " + rightSin);
        System.out.println("Front Cos: " + frontCos);
        System.out.println("Front Sin: " + frontSin);
        System.out.println("Back Cos: " + backCos);
        System.out.println("Back Sin: " + backSin);

        //Going to use a ton of if statements here until I figure out how to be smart.
        //Beginning of 3 wall cases
        if(left < 60 && right < 60 && Math.abs(leftCos - rightCos) < confidence) {
            poseY = (leftCos + rightCos) / 2;
            poseX = back > 60 ? (front > 60 ? poseX : frontCos) : backCos; //Later maybe switch to comparing to odo.
        }
        else if(left < 60 && right < 60 && Math.abs(leftSin - rightSin) < confidence) {
            poseY = back > 60 ? (front > 60 ? poseX : frontSin) : backSin; //Later maybe switch to comparing to odo.
            poseX = (leftSin + rightSin) / 2;
        }
        else if(front < 60 && back < 60 && Math.abs(frontCos - backCos) < confidence) {
            poseY = left > 60 ? (right > 60 ? poseX : rightCos) : leftCos; //Later maybe switch to comparing to odo.
            poseX = (frontCos + backCos) / 2;
        }
        else if(front < 60 && back < 60 && Math.abs(frontSin - backSin) < confidence) {
            poseY = (frontSin + backSin) / 2;
            poseX = left > 60 ? (right > 60 ? poseX : rightSin) : leftSin; //Later maybe switch to comparing to odo.
        }
        //Beginning of corner cases
        else if(left < 60 && right < 60) {
            if(front < 60) {
                if(Math.abs(frontCos - rightSin) < confidence) {
                    poseY = leftCos;
                    poseX = (frontCos + rightSin) / 2;
                }
                else if(Math.abs(frontCos - leftSin) < confidence) {
                    poseY = rightCos;
                    poseX = (frontCos + leftSin) / 2;
                }
                else if(Math.abs(frontSin - leftCos) < confidence) {
                    poseY = (frontSin + leftCos) / 2;
                    poseX = rightSin;
                }
                else if(Math.abs(frontSin - rightCos) < confidence) {
                    poseY = (frontSin + rightCos) / 2;
                    poseX = leftSin;
                }
            }
            else if(back < 60) {
                if(Math.abs(backCos - rightSin) < confidence) {
                    poseY = leftCos;
                    poseX = (backCos + rightSin) / 2;
                }
                else if(Math.abs(backCos - leftSin) < confidence) {
                    poseY = rightCos;
                    poseX = (backCos + leftSin) / 2;
                }
                else if(Math.abs(backSin - leftCos) < confidence) {
                    poseY = (backSin + leftCos) / 2;
                    poseX = rightSin;
                }
                else if(Math.abs(backSin - rightCos) < confidence) {
                    poseY = (backSin + rightCos) / 2;
                    poseX = leftSin;
                }
            }
            //If neither sees, we can't figure it out, so just use odo input.
        }
        else if(front < 60 && back < 60) {
            if(left < 60) {
                if(Math.abs(leftCos - frontSin) < confidence) {
                    poseY = (leftCos + frontSin) / 2;
                    poseX = backCos;
                }
                else if(Math.abs(leftCos - backSin) < confidence) {
                    poseY = (leftCos + backSin) / 2;
                    poseX = frontCos;
                }
                else if(Math.abs(leftSin - frontCos) < confidence) {
                    poseY = backSin;
                    poseX = (leftSin + frontCos) / 2;
                }
                else if(Math.abs(leftSin - backCos) < confidence) {
                    poseY = frontSin;
                    poseX = (leftSin + backCos) / 2;
                }
            }
            else if(right < 60) {
                System.out.println("Yate");
                if(Math.abs(rightCos - frontSin) < confidence) {
                    poseY = (rightCos + frontSin) / 2;
                    poseX = backCos;
                }
                else if(Math.abs(rightCos - backSin) < confidence) {
                    poseY = (rightCos + backSin) / 2;
                    poseX = frontCos;
                }
                else if(Math.abs(rightSin - frontCos) < confidence) {
                    poseY = backSin;
                    poseX = (rightSin + frontCos) / 2;
                }
                else if(Math.abs(rightSin - backCos) < confidence) {
                    poseY = frontSin;
                    poseX = (rightSin + backCos) / 2;
                }
            }
            //If neither sees, we can't figure it out, so just use odo input.
        }
        else {
            if(Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2) {
                poseY = imuHeading < Math.PI / 4 ? (left < right ? leftCos : rightCos) : (front < back ? frontSin : backSin);
                poseX = imuHeading < Math.PI / 4 ? (front < back ? frontCos : backCos) : (left < right ? leftSin : rightSin);
                System.out.println("Yeet");
            }
            else {
                poseY = Math.abs(imuHeading) > 3 * Math.PI / 4 ? (left < right ? leftCos : rightCos) : (front < back ? frontSin : backSin);
                poseX = Math.abs(imuHeading) > 3 * Math.PI / 4 ? (front < back ? frontCos : backCos) : (left < right ? leftSin : rightSin);
            }
        }

        System.out.println("5");
        System.out.println("PoseX: " + poseX + ", Pose Y: " + poseY);

        return new Pose2d(poseX, poseY, imuHeading);
    }

    public Pose2d sensorPose() {
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        config.imu.gettingInput = true;
        config.rightDist.pingSensor();
        config.leftDist.pingSensor();
        config.frontDist.pingSensor();
        config.backDist.pingSensor();
        vals.waitForCycle();
        double imuHeading = config.imu.get()[0];
        config.imu.gettingInput = false;
        double left = config.leftDist.getDistance(DistanceUnit.INCH);
        double right = config.rightDist.getDistance(DistanceUnit.INCH);
        double front = config.frontDist.getDistance(DistanceUnit.INCH);
        double back = config.backDist.getDistance(DistanceUnit.INCH);
        System.out.println("Input : " + left);
        System.out.println("Right input: " + right);
        System.out.println("Front input : " + front);
        System.out.println("Back input: " + back);
        double correctedSideAngle = sensorSideAngle; //Accounts for X vs. Y.
        double correctedStraightAngle = sensorStraightAngle; //Accounts for X vs. Y.
        double correctedHeading = imuHeading > 0 ? (imuHeading + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4 : -((Math.abs(imuHeading) + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4); //Correct heading in each quadrant, as a new quadrant switches what wall it should be seeing.
        left *= (left < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0; //Gets distance from wall as a straight line
        System.out.println("Distance : " + left);
        right *= (right < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier * rightDistMult : 0;
        front *= (front < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0;
        back *= (back < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0;
        left += (left > 0) ? sensorSideOffset * Math.abs(Math.cos(correctedSideAngle + correctedHeading)) : 200; //First quadrant, deals with small cosine values
        System.out.println("Center Distance : " + left);
        right += (right > 0) ? sensorSideOffset * Math.abs(Math.cos(-correctedSideAngle + correctedHeading)) : 200; //Second quadrant, deals with small cosine values
        System.out.println("Right : " + right);
        front += (front > 0) ? sensorStrightOffset * Math.abs(Math.cos(correctedStraightAngle + correctedHeading)) : 200; //First quadrant, deals with small cosine values
        System.out.println("Front center : " + front);
        back += (back > 0) ? sensorStrightOffset * Math.abs(Math.cos(-correctedStraightAngle + correctedHeading)) : 200; //Second quadrant, deals with small cosine values
        System.out.println("Back center: " + back);

        double distanceYLeft = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? left : front) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? right : back); //Switches which distance sensor input corresponds to what actual side relative to robot.
        System.out.println("Actual dist : " + distanceYLeft);
        double distanceYRight = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? right : back) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? left : front);
        System.out.println("Actual right : " + distanceYRight);
        double distanceXFront = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? front : right) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? back : left);
        System.out.println("Actual front : " + distanceXFront);
        double distanceXBack = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? back : left) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? front : right);
        System.out.println("Actual back : " + distanceXBack);

        Pose2d currentPose = config.getPoseEstimate();
        double poseX = currentPose.getX(), poseY = currentPose.getY();

        poseY = (distanceYRight < distanceYLeft) ? - 87 + Math.abs(distanceYRight) : (distanceYLeft < 100) ? 9 - Math.abs(distanceYLeft) : poseY; //Sets up 0 when robot jammed against left wall, grabs smaller of two distances.
        poseX = (distanceXBack < distanceXFront) ? - 135 + Math.abs(distanceXBack) : (distanceXFront < 100) ? 9 - Math.abs(distanceXFront) : poseX; //Sets up 0 when robot jammed against front wall
        System.out.println("Pose Y: " + poseY);
        System.out.println("Pose X: " + poseX);

        return new Pose2d(poseX, poseY, imuHeading);
    }

    public void turnUntilHighGoal(int threshold) {
        while(Math.abs(upperCameraCenter - targetHighGoalX) > threshold) {
            if(upperCameraCenter == 0) break;
            double p = 0.0005, f = 0.02;
            double power = p * (upperCameraCenter - targetHighGoalX);
            power += power > 0 ? f : -f;
            drive.setPower(0, 0, power);
        }
        drive.setPower(0, 0, 0);
    }

    public void turnUntilHighGoalFast(int threshold) {
        while(Math.abs(upperCameraCenter - targetHighGoalX) > threshold) {
            System.out.println("Center: " + upperCameraCenter);
            if(upperCameraCenter == 0) break;
            double p = highGoalFastP, f = highGoalFastF;
            double power = p * (upperCameraCenter - targetHighGoalX);
            power += power > 0 ? f : -f;
            drive.setPower(0, 0, power);
            vals.waitForCycle();
        }
        drive.setPower(0, 0, 0);
    }

    public void turnToPowershot(int pixel) {
        while(Math.abs(maxX - pixel) > 5) {
            if(maxX == -1) break; //If no value
            double p = 0.0004, f = 0.05;
            double power = p * (maxX - pixel);
            power += power > 0 ? f : -f;
            drive.setPower(0, 0, power);
            System.out.println("Max X: " + maxX + ", power: " + power);
            vals.waitForCycle();
        }
        drive.setPower(0, 0, 0);
    }

    public void shootOnce() {
        config.loader.set(load);
        sleep(450);
        config.loader.set(reload);
    }

    public void setPower(double x, double y, double a){
        config.leftRear.setPower(x + y + a);
        config.leftFront.setPower(-x + y + a);
        config.rightFront.setPower(x + y - a);
        config.rightRear.setPower(-x + y - a);
    }

    static class upperCameraPipeline extends OpenCvPipeline
    {

        Mat inputMat = new Mat();
        Mat grayMat = new Mat();
        Mat interMat = new Mat();
        Mat outputMat = new Mat();
        Mat hierarchy = new Mat();

        enum Stage
        {
            INPUT,
            INTER,
            OUTPUT
        }

        private upperCameraPipeline.Stage stageToRenderToViewport = upperCameraPipeline.Stage.INTER;
        private upperCameraPipeline.Stage[] stages = upperCameraPipeline.Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputMat = input;
            Mat rota = Imgproc.getRotationMatrix2D(new org.opencv.core.Point(160, 120), rotateAngle,1);
            Imgproc.warpAffine(inputMat, inputMat, rota, new Size(320,240));
            Imgproc.cvtColor(inputMat, interMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(interMat, interMat, extract);
            Imgproc.medianBlur(interMat, interMat, 5);
            grayMat = interMat.clone();
            Imgproc.threshold(interMat, interMat, redThresh, 255, Imgproc.THRESH_BINARY);
            Imgproc.cvtColor(interMat, outputMat, Imgproc.COLOR_GRAY2RGB);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(interMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            org.opencv.core.Point[] centers = new org.opencv.core.Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new org.opencv.core.Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(outputMat, contoursPolyList, i, new Scalar(0,255,0), 3);
                Imgproc.rectangle(outputMat, boundRect[i].tl(), boundRect[i].br(), new Scalar(255,0,0), 2);
                Imgproc.line(outputMat, new org.opencv.core.Point((boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x)/2, 0), new org.opencv.core.Point((boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x)/2, 480), new Scalar(0,0,255), 3);
                //Imgproc.putText(outputMat, "Points: " + contoursPoly[i].rows(), boundRect[i].tl(), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
            }

            upperCameraCenter = contours.size() >= 1 ? (int) (boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x) : 0;

            maxX = -1;
            ArrayList<Double> pshotsX = new ArrayList<>();
            /*for(int i = 0; i<contours.size()-1; i++){
                if(boundRect[i].tl().x > maxX && boundRect[i].br().y < 230){
                    maxX = boundRect[i].tl().x;
                }
            }
             */
            //minX = boundRect[n].tl().x;
            for(int i = 0; i < contours.size()-1; i++) {
                if(Math.abs(boundRect[i].tl().y - height) < 15 && boundRect[i].tl().x < boundRect[contours.size() - 1].tl().x - leftOff) {
                    if(pshotsX.size() == 1 && boundRect[i].tl().x > pshotsX.get(0))
                        pshotsX.add(0, boundRect[i].tl().x);
                    else if(pshotsX.size() == 2 && boundRect[i].tl().x > pshotsX.get(1))
                        if(boundRect[i].tl().x > pshotsX.get(0)) pshotsX.add(0, boundRect[i].tl().x);
                        else pshotsX.add(1, boundRect[i].tl().x);
                    else pshotsX.add(boundRect[i].tl().x);
                }
            }

            if(pshotsX.size() > 0) {
                maxX = pshotsX.get(0);
                Imgproc.line(outputMat, new org.opencv.core.Point(maxX, 100), new org.opencv.core.Point(maxX, 480), new Scalar(255, 0, 255), 2);
                Imgproc.putText(outputMat, "Powershot Corner: " + maxX, new org.opencv.core.Point(maxX, 100), Imgproc.FONT_HERSHEY_DUPLEX, 0.3, new Scalar(255, 255, 255));
            /*Imgproc.putText(outputMat, "0: " + boundRect[0].tl().y, new org.opencv.core.Point(boundRect[0].tl().x, 100), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255,255,255));
            Imgproc.line(outputMat, new org.opencv.core.Point(boundRect[0].tl().x, 100), new org.opencv.core.Point(boundRect[0].tl().x, 480),new Scalar(255,0,255), 2);
            Imgproc.putText(outputMat, "1: " + boundRect[1].tl().y, new org.opencv.core.Point(boundRect[1].tl().x, 100), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255,255,255));
            Imgproc.line(outputMat, new org.opencv.core.Point(boundRect[1].tl().x, 100), new org.opencv.core.Point(boundRect[1].tl().x, 480),new Scalar(255,0,255), 2);
            Imgproc.putText(outputMat, "2: " + boundRect[2].tl().y, new org.opencv.core.Point(boundRect[2].tl().x, 100), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255,255,255));
            Imgproc.line(outputMat, new org.opencv.core.Point(boundRect[2].tl().x, 100), new org.opencv.core.Point(boundRect[2].tl().x, 480),new Scalar(255,0,255), 2);
            Imgproc.putText(outputMat, "3: " + boundRect[3].tl().y, new org.opencv.core.Point(boundRect[3].tl().x, 100), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255,255,255));
            Imgproc.line(outputMat, new org.opencv.core.Point(boundRect[3].tl().x, 100), new org.opencv.core.Point(boundRect[3].tl().x, 480),new Scalar(255,0,255), 2);
*/
            }

            if (contours.size() >= 1) Imgproc.putText(outputMat, "Center: " + (boundRect[contours.size() - 1].tl().x + boundRect[contours.size() - 1].br().x), boundRect[contours.size() - 1].tl(), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255, 255, 255));

            switch (stageToRenderToViewport){
                case INPUT:
                {
                    return inputMat;
                }
                case INTER:
                {
                    return outputMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}
