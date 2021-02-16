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
    public static int sampleHeight = 3;
    public static org.opencv.core.Point topCenter = new org.opencv.core.Point(510, 420);
    public static org.opencv.core.Point bottomCenter = new org.opencv.core.Point(510, 350);
    public static org.opencv.core.Point leftBar1 = new org.opencv.core.Point(442, 360), leftBar2 = new org.opencv.core.Point(451, 436), rightBar1 = new org.opencv.core.Point(198, 359), rightBar2 = new org.opencv.core.Point(207, 435);
    public static int thresh = 140, redThresh = 137;
    public static int wobbleThresh = 145, initThresh = 133;
    public static int stackSize = -1;
    private static double color1, color2;
    public static boolean initDetect = true, lameMode = true;
    public static boolean properSetup = false;
    public static double offsetDivisor = 50;
    public static double rotateAngle = 195;

    public static int extract = 1;
    public static int row = 320;

    OpenCvCamera webCam, webcam2;


    ConfigurationRR config;
    DriveObjectRobotMovement drive;
    ValueStorage vals = new ValueStorage();
    HardwareThread hardware;
    Thread returnToShoot, powerShots, shootMacro, grabWobble, dropWobble, lowerWobble;
    public static double wobbleUp = 0.22, wobbleDown = 0.65, wobbleMid = 0.45, gripperOpen = 0, gripperClosed = 1, load = 0.5, reload = 0.13, shooterSpeed = -1640, multiplier = 0.97, sensorSideOffset = 8.20, sensorSideAngle = 0.66, sensorStrightOffset = 8, sensorStrightAngle = 0; //Sheets had 1.15 as multiplier, seeing if just my house that's off

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
            //leSense = hardwareMap.get(DistanceSensor.class, "distanceLeft");
            config = new ConfigurationRR(hardwareMap);
            hardware = new HardwareThread(hardwareMap, vals, config);
            hardware.start();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

            webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), cameraMonitorViewId);
            webcam2.openCameraDevice();//open camera
            webcam2.setPipeline(new upperCameraPipeline());//different stages
            webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC
            //width, height
            //width = height in this case, because camera is in portrait mode.
            //config.Configure(hardwareMap, vals);
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
        config.imu.retrievingHardware(true);
        //config.leftDist.pingSensor();
        //config.rightDist.pingSensor();
        //config.frontDist.pingSensor();
        //config.backDist.pingSensor();
        config.setPoseEstimate(sensorPose());
        telemetry.addData("Pose: ", config.getPoseEstimate());
        double head = config.imu.get()[0];
        telemetry.addData("Heading: ", config.imu.get()[0]);
        //double dist = config.leftDist.getDistance(DistanceUnit.MM);
        //dist *= multiplier; //(dist > 800) ? multiplier : 1;
        //dist *= Math.cos(head);
        //telemetry.addData("Center distance: ", dist / 25.4 + Math.cos(head) * sensorXOffset + Math.sin(sensorYOffset));
        //telemetry.addData("Distance: ", dist / 25.4);
        telemetry.update();
        if(gamepad2.a && !pressedShooter) {
            pressedShooter = true;
            shooterFast = !shooterFast;
        }
        else if(pressedShooter && !gamepad2.a) pressedShooter = false;
        if(gamepad2.start) {
            config.setPoseEstimate(new Pose2d(0, 0, 0));
            config.imu.resetIMU();
        }
        if(gamepad2.left_stick_y > 0.5) shooterSpeed = -1540;
        else if(gamepad2.left_stick_y < -0.5) shooterSpeed = -1640;
        config.shooter.set(shooterFast ? shooterSpeed : 0);
        //if(gamepad1.start) { //Will be changed later
            //config.imu.resetIMU();
            //if(time.seconds() < 5) config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), 0.0));
            //config.setPoseEstimate(new Pose2d(-63, -63, Math.PI)); //In corner, AKA after dropping off wobble goal.
        //}
        if(gamepad1.start && !shootMacro.isAlive()) {
            returnToShoot.start();
            //config.setPoseEstimate(config.getPoseEstimate());
            /*Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), header), new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
             */
            while(returnToShoot.isAlive());
        }
        else if(gamepad1.a && !shootMacro.isAlive() && config.ingester.get()[0] >= shooterSpeed - 500) { //Just making sure we don't shoot until after shooter reaches speed, may add this check to the macro.
            shootMacro.start();
            /*drive.runWithEncoder(false);
            config.motors.get(0).reverse(false);
            config.motors.get(1).reverse(false);
            config.motors.get(2).reverse(true);
            config.motors.get(3).reverse(true);
            Pose2d currentPose = config.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
            while((Math.hypot(Math.abs(highGoalShoot.getX()-currentPose.getX()),Math.abs(highGoalShoot.getY()-currentPose.getY()))>0.75)&&!isStopRequested()){
                config.update();
                currentPose = config.getPoseEstimate();
                drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
                telemetry.addData("Position: ", currentPose);
                telemetry.addData("Pose: ", drive.robotPosition + " " + drive.worldAngle);
                telemetry.update();
                drive.goToPosition(highGoalShoot.getX(),highGoalShoot.getY(),0.8,270,0.7);
            }
            drive.setPower(0,0,0);
            sleep(200);
            double imuHeading = config.imu.get()[0];
            while((Math.abs(imuHeading)>Math.toRadians(2)) && !isStopRequested()){
                config.update();
                imuHeading = config.imu.get()[0];
                int invert = imuHeading > 180 ? -1 : 1;
                double p = 0.2, f = 0.15;
                drive.setPower(0,0, invert * (f+(p*Math.abs(imuHeading))));
            }
            drive.runWithEncoder(true);
            config.motors.get(0).reverse(true);
            config.motors.get(1).reverse(true);
            config.motors.get(2).reverse(false);
            config.motors.get(3).reverse(false);
             */
        }
        else if(gamepad1.back && !shootMacro.isAlive()) {
            //config.setPoseEstimate(config.getPoseEstimate());
            //config.turn(Math.toRadians(12));
            //powerShots.start();
            //while(powerShots.isAlive());
        }
        System.out.println("Time 1: " + time.milliseconds());
        double speedMultiplier = -1;
        if(gamepad1.left_bumper) speedMultiplier = -0.2;
        //Pose2d currentPose = config.getPoseEstimate();
        //double arctan = Math.atan(currentPose.getY() / currentPose.getX());
        //int inverse = ((2 * Math.PI - head) + arctan) % (2 * Math.PI) > Math.PI ? 1 : -1;
        //double angPower = -0.4 * inverse * ((Math.abs(head - arctan)) > Math.PI ? (Math.abs((head > Math.PI ? 2 * Math.PI : 0) - head) + Math.abs((arctan > Math.PI ? 2 * Math.PI : 0) - arctan)) : Math.abs(head - arctan)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
        //angPower *= (Math.abs(arctan - head) < 1) ? 0 : 1;
        //telemetry.addData("Ang power: ", angPower);
        //telemetry.update();
        //setPower(multiplier * gamepad1.left_stick_x, multiplier * gamepad1.left_stick_y, multiplier * gamepad1.right_stick_x);
        if(gamepad1.right_bumper) setPower(speedMultiplier * (gamepad1.left_stick_x * Math.cos(head) + gamepad1.left_stick_y * Math.sin(head)), speedMultiplier * (gamepad1.left_stick_x * Math.sin(head) + gamepad1.left_stick_y * Math.cos(head)), -speedMultiplier * gamepad1.right_stick_x); //Should be field-centric
        else setPower(speedMultiplier * gamepad1.left_stick_x, speedMultiplier * gamepad1.left_stick_y, -speedMultiplier * gamepad1.right_stick_x);
        if(!pressedOdoAdjust && (gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down)) {
            //currentPose = config.getPoseEstimate();
            //config.setPoseEstimate(new Pose2d(currentPose.getX() + (gamepad2.dpad_up ? -2 : (gamepad2.dpad_down ? 2 : 0)), currentPose.getY() + (gamepad2.dpad_left ? -2 : (gamepad2.dpad_right ? 2 : 0)), currentPose.getHeading()));
            pressedOdoAdjust = true;
        }
        else if(pressedOdoAdjust && !(gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up || gamepad2.dpad_down)) pressedOdoAdjust = false;
        if(shootMacro.isAlive()) {}
        else if(gamepad2.back && !pressedLock) {
            lockedLoader = !lockedLoader;
            pressedLock = true;
        }
        else if(pressedLock && !gamepad1.right_bumper && !gamepad2.back) pressedLock = false;
        else if(gamepad1.b || lockedLoader && config.ingester.get()[0] >= shooterSpeed - 500) config.loader.set(load);
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

        if(gamepad2.right_stick_button) intakeSpeed = -1;
        else if(gamepad2.b) intakeSpeed = 0;
        else if(gamepad2.left_stick_button) intakeSpeed = 1;
        config.ingester.set(intakeSpeed);
        config.preIngest.set(intakeSpeed);

        System.out.println("Time 2: " + time.milliseconds());

        if(gamepad1.right_stick_button) {
            config.imu.retrievingHardware(true);
            sleep(40);
            double imuHeading = config.imu.get()[0];
            while((Math.abs(imuHeading)>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
                config.update();
                imuHeading = config.imu.get()[0];
                if(imuHeading < 0) imuHeading += 2 * Math.PI;
                //currentPose = drive.getPoseEstimate();
                double p = 0.35, f = 0.04;
                int invert = ((2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
                double power = invert * p * (Math.abs(imuHeading) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((0 > Math.PI ? 2 * Math.PI : 0) - 0)) : Math.abs(imuHeading - 0)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
                power += (power > 0 ? f : -f);
                drive.setPower(0, 0, -power);
            }
            drive.setPower(0, 0, 0);
            //currentPose = config.getPoseEstimate();
            //config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
            config.imu.retrievingHardware(false);
        }
        System.out.println("Time 3: " + time.milliseconds());
    }

    public void configureMacros() {
        /*Sequence returnToHighGoal = new Sequence(() -> {
            config.setPoseEstimate(config.getPoseEstimate());
            Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), header), new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
            return null;
        });
         */
        Sequence returnToHighGoal = new Sequence(() -> {
            //imuTurn();
            //Pose2d currentPose = config.getPoseEstimate();
            //config.setPoseEstimate(currentPose.getX(), leSense.getDistance(DistanceUnit.INCH));
            roadRunnerToPosition(highGoalShoot, 0.8);
            highGoalWallAdjust();
            return null;
        });
        Sequence returnToHighGoalDistance = new Sequence(() -> {
            //Pose2d currentPose = config.getPoseEstimate();
            config.setPoseEstimate(sensorPose());
            System.out.println("Pose: " + config.getPoseEstimate());
            roadRunnerToPosition(new Pose2d(-78, -54), 0.5);
            imuTurn();
            config.setPoseEstimate(sensorPose());
            //config.rightDist.pingSensor();
            //config.leftDist.pingSensor();
            //sleep(80);
            //config.setPoseEstimate(sensorPoseTwoSensor());
            return null;
        });
        Sequence tripleShoot = new Sequence(() -> {
            config.shooter.set(-1640);
            for(int i = 0; i < 3; i++) {
                shootOnce();
                config.shooter.set(-1580);
                sleep(300);
            }
            config.loader.set(load);
            return null;
        }, returnToHighGoalDistance);
        returnToShoot = new Thread(tripleShoot);

        Sequence shootThrice = new Sequence(() -> {
            if(lockedLoader) {
                config.loader.set(reload);
                lockedLoader = false;
                sleep(300);
            }
            for(int i = 0; i < 2; i++) {
                shootOnce();
                sleep(300);
            }
            shootOnce();
            config.loader.set(load);
            return null;
        });
        shootMacro = new Thread(shootThrice);

        /*Sequence returnToPowerShot = new Sequence(() -> {
            config.setPoseEstimate(config.getPoseEstimate());
            Trajectory goToShoot = config.trajectoryBuilder(config.getPoseEstimate())
                    .lineToLinearHeading(powerShotShoot, new DriveConstraints(
                            45.0, 30.0, 0.0,
                            Math.toRadians(120.0), Math.toRadians(120.0), 0.0))
                    .build();
            config.followTrajectory(goToShoot);
            return null;
        });
         */
        Sequence returnToPowerShot = new Sequence(() -> {
            purePursuitToPosition(powerShotShoot, 0.7);
            return null;
        });
        Sequence pivotShoot = new Sequence(() -> {
            config.shooter.set(-1600);
            for(int i = 0; i < 2; i++) {
                shootOnce();
                config.turn(Math.toRadians(-8));
                sleep(400);
            }
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

    public void purePursuitToPosition(Pose2d targetPose, double speed) {
        //drive.runWithEncoder(false);
        config.motors.get(0).reverse(false);
        config.motors.get(1).reverse(false);
        config.motors.get(2).reverse(true);
        config.motors.get(3).reverse(true);
        Pose2d currentPose = config.getPoseEstimate();
        drive.updatePose(new Point(currentPose.getX(), currentPose.getY()), currentPose.getHeading());
        while((Math.hypot(Math.abs(targetPose.getX()-currentPose.getX()),Math.abs(targetPose.getY()-currentPose.getY()))>0.5)&&!isStopRequested()){
            config.update();
            currentPose = config.getPoseEstimate();
            drive.updatePose(new Point(currentPose.getX(), currentPose.getY()),currentPose.getHeading());
            //t.addData("Current Position (Odo): ", currentPose);
            //t.addData("Target Point: ", targetPose);
            //t.update();
            drive.goToPosition(targetPose.getX(),targetPose.getY(),speed,90,0.3);
        }
        drive.setPower(0,0,0);
        sleep(500);
        if(isStopRequested()) return;
        config.imu.retrievingHardware(true);
        sleep(200);
        double imuHeading = config.imu.get()[0];
        while((Math.abs(imuHeading - targetPose.getHeading())>Math.toRadians(1)) && !isStopRequested() && opModeIsActive()){
            config.update();
            imuHeading = config.imu.get()[0];
            if(imuHeading < 0) imuHeading += 2 * Math.PI;
            //currentPose = drive.getPoseEstimate();
            double p = 0.3, f = 0.07;
            int invert = (targetPose.getHeading() + (2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double power = invert * p * (Math.abs(imuHeading - targetPose.getHeading()) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((targetPose.getHeading() > Math.PI ? 2 * Math.PI : 0) - targetPose.getHeading())) : Math.abs(imuHeading - targetPose.getHeading())); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
        }
        if(isStopRequested()) return;
        config.imu.retrievingHardware(false);
        double distance = leSense.getDistance(DistanceUnit.INCH);
        currentPose = config.getPoseEstimate();
        if(distance > distanceLeft || distance < distanceRight) {
            Trajectory traj = config.trajectoryBuilder(currentPose)
                    .strafeTo(new Vector2d(currentPose.getX(), currentPose.getY() - distance + 20))
                    .build();
            config.followTrajectory(traj);
        }
        if(isStopRequested()) return;
        drive.setPower(0, 0, 0);
        sleep(200);
        //drive.runWithEncoder(true);
        config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
    }

    public void imuTurn() {
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

    public void roadRunnerToPosition(Pose2d targetPose, double speed) {
        /*config.motors.get(0).reverse(false);
        config.motors.get(1).reverse(false);
        config.motors.get(2).reverse(true);
        config.motors.get(3).reverse(true);
         */
        drive.runWithEncoder(true);
        Pose2d currentPose = config.getPoseEstimate();
        Trajectory traj = config.trajectoryBuilder(currentPose)
                .strafeTo(targetPose.vec())
                .build();
        config.followTrajectory(traj);
        drive.setPower(0,0,0);
        drive.runWithEncoder(false);
        /*config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
         */
        /*config.imu.retrievingHardware(true);
        sleep(40);
        double imuHeading = config.imu.get()[0];
        while((Math.abs(imuHeading - targetPose.getHeading())>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
            config.update();
            imuHeading = config.imu.get()[0];
            if(imuHeading < 0) imuHeading += 2 * Math.PI;
            //currentPose = drive.getPoseEstimate();
            double p = 0.35, f = 0.04;
            int invert = (targetPose.getHeading() + (2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double power = invert * p * (Math.abs(imuHeading - targetPose.getHeading()) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((targetPose.getHeading() > Math.PI ? 2 * Math.PI : 0) - targetPose.getHeading())) : Math.abs(imuHeading - targetPose.getHeading())); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setPower(0, 0, power);
        }
        if(isStopRequested()) return;
        drive.setPower(0, 0, 0);
        currentPose = config.getPoseEstimate();
        imuHeading = config.imu.get()[0];
        config.imu.retrievingHardware(false);
        config.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
         */

    }

    private void highGoalWallAdjust() {
        double distance = leSense.getDistance(DistanceUnit.INCH);
        System.out.println("Distance sensor: " + distance);
        if((distance > distanceLeft || distance < distanceRight) && distance < 300) {
            Pose2d currentPose = config.getPoseEstimate();
            Trajectory correct = config.trajectoryBuilder(config.getPoseEstimate())
                    .strafeRight(distance - wallDistance)
                    .build();
            config.followTrajectory(correct);
            config.imu.retrievingHardware(true);
            sleep(40);
            double imuHeading = config.imu.get()[0];
            config.setPoseEstimate(new Pose2d(currentPose.getX(), distance - wallDistance, imuHeading));
            while((Math.abs(imuHeading)>Math.toRadians(0.5)) && !isStopRequested() && opModeIsActive()){
                config.update();
                imuHeading = config.imu.get()[0];
                if(imuHeading < 0) imuHeading += 2 * Math.PI;
                //currentPose = drive.getPoseEstimate();
                double p = 0.35, f = 0.04;
                int invert = ((2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
                double power = invert * p * (Math.abs(imuHeading) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((0 > Math.PI ? 2 * Math.PI : 0) - 0)) : Math.abs(imuHeading - 0)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
                power += (power > 0 ? f : -f);
                drive.setPower(0, 0, power);
            }
            drive.setPower(0, 0, 0);
            config.imu.retrievingHardware(false);
        }
        if(isStopRequested()) return;
        drive.runWithEncoder(false);
        config.motors.get(0).reverse(true);
        config.motors.get(1).reverse(true);
        config.motors.get(2).reverse(false);
        config.motors.get(3).reverse(false);
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
        vals.waitForCycle();
        double imuHeading = config.imu.get()[0];
        config.imu.gettingInput = false;
        double left = config.leftDist.getDistance(DistanceUnit.INCH);
        double right = config.rightDist.getDistance(DistanceUnit.INCH);
        double front = config.frontDist.getDistance(DistanceUnit.INCH);
        double back = config.backDist.getDistance(DistanceUnit.INCH);
        double correctedSideAngle = sensorSideAngle; //Accounts for X vs. Y.
        double correctedStraightAngle = sensorStrightAngle; //Accounts for X vs. Y.
        double correctedHeading = imuHeading > 0 ? (imuHeading + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4 : -((Math.abs(imuHeading) + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4);
        if(Math.abs(imuHeading) > Math.PI / 4 && Math.abs(imuHeading) < 3 * Math.PI / 4) {
            //correctedHeading = Math.PI / 2 - correctedHeading;
            //correctedSideAngle = Math.PI / 2 - sensorSideAngle;
            //correctedStraightAngle = Math.PI / 2 - sensorStrightAngle;
        }
        left *= (left < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 1 / left * 200;
        right *= (right < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 1 / right * 200;
        front *= (front < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 1 / front * 200;
        back *= (back < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 1 / back * 200;
        System.out.println("Left Before: " + left);
        System.out.println("Right Before: " + right);
        System.out.println("Front Before: " + front);
        System.out.println("Back Before: " + back);
        left += (left != 200) ? sensorSideOffset * Math.abs(Math.cos(correctedSideAngle + correctedHeading)) : 0; //First quadrant, deals with small cosine values
        right += (right != 200) ? sensorSideOffset * Math.abs(Math.cos(-correctedSideAngle + correctedHeading)) : 0; //Second quadrant, deals with small cosine values
        front += (front != 200) ? sensorStrightOffset * Math.abs(Math.cos(correctedStraightAngle + correctedHeading)) : 0; //First quadrant, deals with small cosine values
        back += (back != 200) ? sensorStrightOffset * Math.abs(Math.cos(-correctedStraightAngle + correctedHeading)) : 0; //Second quadrant, deals with small cosine values
        System.out.println("Left: " + left);
        System.out.println("Right: " + right);
        System.out.println("Front: " + front);
        System.out.println("Back: " + back);

        double distanceYLeft = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? left : front) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? right : back);
        double distanceYRight = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? right : back) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? left : front);
        double distanceXFront = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? front : right) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? back : left);
        double distanceXBack = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? back : left) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? front : right);

        Pose2d currentPose = config.getPoseEstimate();
        double poseX = currentPose.getX(), poseY = currentPose.getY();

        poseY = (distanceYRight < distanceYLeft) ? - 87 + Math.abs(distanceYRight) : (distanceYLeft < 100) ? 9 - Math.abs(distanceYLeft) : poseY; //Sets up 0 when robot jammed against left wall
        poseX = (distanceXBack < distanceXFront) ? - 131 + Math.abs(distanceXBack) : (distanceXFront < 100) ? 9 - Math.abs(distanceXFront) : poseX; //Sets up 0 when robot jammed against front wall

        return new Pose2d(poseX, poseY, imuHeading);
    }

    public Pose2d sensorPoseTwoSensor() {
        //Assumes the sensors are updated
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        //DO NOT CHANGE THIS METHOD, this is our fallback if something does not work/reference for what works.

        double imuHeading = config.imu.get()[0];
        double left = config.leftDist.getDistance(DistanceUnit.INCH);
        left = (left < 100) ? left * Math.abs(Math.cos(imuHeading)) * multiplier + (Math.abs(imuHeading) < Math.PI / 2 ? sensorSideOffset * Math.cos(sensorSideAngle + imuHeading) : -sensorSideOffset * Math.cos(sensorSideAngle + imuHeading)) : 200; //First quadrant, deals with small cosine values
        double right = config.rightDist.getDistance(DistanceUnit.INCH);
        right = (right < 100) ? right * Math.abs(Math.cos(imuHeading)) * multiplier - (Math.abs(imuHeading) < Math.PI / 2 ? sensorSideOffset * Math.cos(Math.PI - sensorSideAngle + imuHeading) : -sensorSideOffset * Math.cos(Math.PI - sensorSideAngle + imuHeading))  : 200; //Second quadrant, deals with small cosine values
        //double front = config.frontDist.getDistance(DistanceUnit.INCH);
        //front = (front < 100) ? front * Math.abs(Math.cos(imuHeading)) * multiplier + (Math.abs(imuHeading) < Math.PI / 2 ? sensorSideOffset * Math.cos(sensorSideAngle + imuHeading) : -sensorSideOffset * Math.cos(sensorSideAngle + imuHeading)) : 200; //First quadrant, deals with small cosine values
        //double back = drive.distanceBack.getDistance(DistanceUnit.INCH);
        //back *= Math.abs(Math.cos(imuHeading));

        double distanceYLeft = Math.abs(imuHeading) < Math.PI / 2 ? left : right;
        double distanceYRight = Math.abs(imuHeading) < Math.PI / 2 ? right : left;
        System.out.println("Angle: " + imuHeading + "Distance Right: " + distanceYRight + ", Distance Left: " + distanceYLeft);
        //double distanceXFront = Math.abs(imuHeading - Math.PI) < Math.PI / 2 ? back : front;
        //double distanceXBack = Math.abs(imuHeading - Math.PI) < Math.PI / 2 ? front : back;

        Pose2d currentPose = config.getPoseEstimate();
        double poseX = currentPose.getX(), poseY = currentPose.getY();

        //poseY = (distanceYRight < 100) ? - 98.5 - sensorYOffset + distanceYRight + Math.cos(imuHeading) * sensorYOffset + Math.sin(imuHeading) * sensorXOffset : (distanceYLeft < 100) ? 2.5 + sensorYOffset - (distanceYLeft + Math.cos(imuHeading) * sensorYOffset + Math.sin(imuHeading) * sensorXOffset) : poseY;
        poseY = (distanceYRight < distanceYLeft) ? - 87 + Math.abs(distanceYRight) : (distanceYLeft < 100) ? 9 - Math.abs(distanceYLeft) : poseY; //Sets up 0 when robot jammed against left wall

        //poseX = (distanceXFront < 100) ? 17 - distanceXFront : (distanceXBack < 100) ? - 65 + distanceXBack : poseX;

        return new Pose2d(poseX, poseY, imuHeading);
    }

    public void shootOnce() {
        config.loader.set(load);
        sleep(550);
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
