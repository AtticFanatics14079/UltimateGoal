package org.firstinspires.ftc.teamcode.Autonomous.RRBasicPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilderKt;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.DriveConstants;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TeleOp.UltimateGoalTeleOpV1;
import org.firstinspires.ftc.teamcode.Vision.scanPipeline;
import org.firstinspires.ftc.teamcode.Vision.twoScanPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
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

import javax.xml.transform.sax.SAXSource;

@Config
@Autonomous
public class AllPathsVisionDistance extends LinearOpMode {

    private DriveConstraints verySlow = new DriveConstraints(
            15, 15.0, 0.0,
            Math.toRadians(90.0), Math.toRadians(90.0), 0.0
    );

    private DriveConstraints slow = new DriveConstraints(
            40.0, 25.0, 0.0,
            Math.toRadians(120.0), Math.toRadians(120.0), 0.0
    );

    private DriveConstraints kindaFast = new DriveConstraints(
            50.0, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    private DriveConstraints omegaFast = new DriveConstraints(
            60.0, 55.0, 0.0,
            Math.toRadians(270.0), Math.toRadians(270.0), 0.0
    );

    private DriveConstraints gas = new DriveConstraints(
            80.0, 70.0, 0.0,
            Math.toRadians(360.0), Math.toRadians(360.0), 0.0
    );

    private DriveConstraints omegaGas = new DriveConstraints(
            100.0, 100.0, 0.0,
            Math.toRadians(360.0), Math.toRadians(360.0), 0.0
    );

    private Pose2d startPose = new Pose2d(-135.0, -59.0, Math.toRadians(0.0)); //Conversion of -72.0 on X and -25.0 on Y
    private Pose2d powerShotShoot = new Pose2d(-74.0, -43.0);
    private Pose2d highGoalShoot = new Pose2d(-91.0, -69.0);
    private Pose2d path4dropoff = new Pose2d(-18.0, -79.0);
    private Pose2d pickup4 = new Pose2d(-112, -68, Math.toRadians(-180.0));
    private Pose2d path1dropoff = new Pose2d(-52.0, -61.0);
    private Pose2d pickup1 = new Pose2d(-112, -79, Math.toRadians(-180.0)); //Was x = -38.5, y = -40 before moving to back of tape
    private Pose2d path0dropoff = new Pose2d(-74.0, -84.0);
    private Pose2d pickup0 = new Pose2d(-112, -79, Math.toRadians(-180.0));
    private Pose2d endLocation = new Pose2d(-67.0, -68.0, 0.0);
    private Pose2d park4 = new Pose2d(-62.0, -85.0);
    private Pose2d park1 = new Pose2d(-62.0, -73.0);

    private Pose2d path1point1 = new Pose2d(-102.0, -59.0);

    public static double wobbleUp = 0.4, wobbleDown = 0.85, wobbleMid = 0.67, wobblePushStack = 0.785, gripperOpen = 0, gripperClosed = 1, loaded = 0.48, reload = 0.14, path5highgoalX = -80, path5highgoalY = -60, offsetDivisor = 50;

    public static double multiplier = 0.97, sensorSideOffset = 8.20, sensorSideAngle = 0.66, sensorStrightOffset = 8, sensorStrightAngle = 0, rightDistMult = 1;

    public static int pshotLeft = 64, pshotMid = 12, pshotRight = 10;

    public static int ring1velo = -1640, ring2velo = -1640, ring3velo = -1640;

    private Pose2d powerShotBackShoot = new Pose2d(-102.0,-50.0, 0); //Is actually 1
    private Pose2d powerShot = new Pose2d(-80.0,-30.0, 0); //Is actually 1
    private Pose2d ingestStack = new Pose2d(-79.0, -50.0, Math.toRadians(0));

    public static double powshot1 = 70, powshot2 = 110, powshot3 = 147, minX;

    private final int rows = 640;
    private final int cols = 480;
    public static int sampleWidth = 30;
    public static int sampleHeight = 3;
    public static Point topCenter = new Point(260, 130);
    public static Point bottomCenter = new Point(260, 60);
    public static Point leftBar1 = new Point(456, 350), leftBar2 = new Point(464, 426), rightBar1 = new Point(210, 352), rightBar2 = new Point(222, 422);
    public static int thresh = 130;
    public static int wobbleThresh = 145, initThresh = 120, targetHighGoalX = 180;
    public static int stackSize = -1;
    public static boolean properSetup = false;
    private static double color1, color2;
    public static int upperCameraCenter = 0;
    public static double rotateAngle = 180;
    public static int redThresh = 136;

    public static int extract = 1;
    public static int row = 320;

    private lowerCameraPipeline pipeline = new lowerCameraPipeline();

    public static boolean usingCamera = true, wait = true;

    OpenCvCamera webCam, webcam2;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            drive = new SampleMecanumDrive(hardwareMap);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            if (usingCamera) {
                int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
                int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                        .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

                webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), viewportContainerIds[0]);
                webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), viewportContainerIds[1]);
                webCam.openCameraDevice();//open camera
                webcam2.openCameraDevice();//open camera
                webCam.setPipeline(new lowerCameraPipeline());//different stages
                webcam2.setPipeline(new upperCameraPipeline());//different stages
                webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
                webcam2.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC
                pipeline.initDetect = true;
            }

            drive.loader.setPosition(reload);

            drive.wobble.setPosition(wobbleDown);

            sleep(2000);

            drive.gripper.setPosition(gripperClosed);

            sleep(1000);

            drive.wobble.setPosition(wobbleUp);

            while (!isStarted() && !isStopRequested()) {
                telemetry.addData("Le stack: ", stackSize);
                telemetry.addData("Setup: ", properSetup);
                telemetry.update();
                if(gamepad1.start) {
                    drive.setPoseEstimate(startPose);
                }
            }

            if (isStopRequested()) return;

            int stackSize2 = stackSize;

            drive.setPoseEstimate(startPose);

            drive.shooter.setVelocity(-1640);

            drive.wobble.setPosition(wobbleMid);

            double imuHeading;// = drive.imu.getAngularOrientation().firstAngle;
            Pose2d currentPose = sensorPoseAnalog();

            switch (stackSize2) {
                case 0: {

                    drive.ingester.setPower(0);

                    //drive.setPoseEstimate(currentPose);

                    imuTurn(0);
                    drive.setPoseEstimate(sensorPoseAnalog());
                    System.out.println("Sensor Pose: " + sensorPoseAnalog());
                    System.out.println("Current Pose: " + drive.getPoseEstimate());
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));

                    Trajectory powershots = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(powerShotShoot.vec(), 0.0, slow)
                            .build();
                    drive.followTrajectory(powershots);

                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    //imuTurn(Math.toRadians(2));
                    //imuTurn(Math.toRadians(pshot2));
                    //turnToPowershot(25);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    //imuTurn(Math.toRadians(pshot3));
                    //turnToPowershot(67);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);

                    Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(path0dropoff.vec(), 0, slow)
                            .build();
                    drive.followTrajectory(wobble1);

                    drive.loader.setPosition(reload);

                    //currentPose = drive.getPoseEstimate();
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
                    //drive.setPoseEstimate(sensorPoseAnalog());

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1000);
                    drive.wobble.setPosition(wobbleUp);

                    Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .lineToLinearHeading(new Pose2d(pickup0.getX() + 24, pickup0.getY(), Math.toRadians(180.0)), verySlow)
                            .build();
                    drive.followTrajectory(wobble2);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(600);

                    drive.setPoseEstimate(sensorPoseAnalog());
                    //currentPose = drive.getPoseEstimate();
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() - offset - 1, imuHeading));

                    Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeTo(pickup0.vec())
                            .build();
                    drive.followTrajectory(pickupWobble);

                    //currentPose = drive.getPoseEstimate();
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() + offset + 1, imuHeading));
                    drive.setPoseEstimate(sensorPoseAnalog()); //Should account for offset, so no need for previous line.

                    drive.gripper.setPosition(gripperClosed);
                    sleep(1200);
                    drive.wobble.setPosition(0.6);

                    Trajectory dropoff = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(path0dropoff.getX(), path0dropoff.getY() - 12, 0.0), slow)
                            .build();
                    drive.followTrajectory(dropoff);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1500);

                    drive.wobble.setPosition(wobbleUp);
                    sleep(500);
                }
                break;
                case 1: {

                    drive.ingester.setPower(1);
                    drive.preIngest.setPower(1);
                    drive.shooter.setVelocity(ring3velo);

                    drive.setPoseEstimate(sensorPoseAnalog());

                    Trajectory toStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(path1point1.vec(), Math.toRadians(-10.0))
                            .build();
                    drive.followTrajectory(toStack);

                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1640);

                    Trajectory powerShots = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(powerShotShoot.vec(), 0.0)
                            .build();
                    drive.followTrajectory(powerShots);

                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    //imuTurn(Math.toRadians(2));
                    //imuTurn(Math.toRadians(pshot2));
                    //turnToPowershot(25);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    //imuTurn(Math.toRadians(pshot3));
                    //turnToPowershot(67);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);

                    Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(path1dropoff)
                            .build();
                    drive.followTrajectory(wobble1);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1000);
                    drive.wobble.setPosition(wobbleUp);

                    drive.setPoseEstimate(sensorPoseAnalog());

                    Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .lineToLinearHeading(new Pose2d(pickup1.getX() + 24, pickup1.getY(), Math.toRadians(180.0)), slow)
                            .build();
                    drive.followTrajectory(wobble2);

                    drive.wobble.setPosition(wobbleDown);

                    drive.setPoseEstimate(sensorPoseAnalog());

                    drive.ingester.setPower(0);
                    drive.preIngest.setPower(0);

                    Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(pickup1)
                            .build();
                    drive.followTrajectory(pickupWobble);

                    drive.gripper.setPosition(gripperClosed);
                    sleep(1000);
                    drive.wobble.setPosition(0.55);

                    Trajectory drop = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                            .lineToLinearHeading(new Pose2d(path1dropoff.getX(), path1dropoff.getY() - 12, Math.toRadians(0.0)), kindaFast)
                            .build();
                    drive.followTrajectory(drop);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1200);
                    drive.wobble.setPosition(wobbleUp);

                    Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                            .strafeTo(park1.vec())
                            .build();
                    drive.followTrajectory(toPark);
                }
                break;
                case 4: {

                    drive.ingester.setPower(1);
                    drive.preIngest.setPower(1);
                    drive.shooter.setVelocity(ring3velo);

                    drive.setPoseEstimate(sensorPoseAnalog());

                    Trajectory toStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(path1point1.vec(), Math.toRadians(-10.0))
                            .build();
                    drive.followTrajectory(toStack);

                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1640);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1640);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);

                    Trajectory pickupStack = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(powerShotShoot.getX() - 20, powerShotShoot.getY() - 20), Math.toRadians(15.0))
                            .addDisplacementMarker(() -> {drive.loader.setPosition(loaded);})
                            .splineTo(powerShotShoot.vec(), 0.0)
                            .addDisplacementMarker(() -> {drive.loader.setPosition(reload);})
                            .build();
                    drive.followTrajectory(pickupStack);

                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    //imuTurn(Math.toRadians(2));
                    //imuTurn(Math.toRadians(pshot2));
                    //turnToPowershot(25);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    //imuTurn(Math.toRadians(pshot3));
                    //turnToPowershot(67);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(500);
                    drive.loader.setPosition(reload);

                    Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(path4dropoff.getX(), path4dropoff.getY() - 6), 0.0)
                            //.splineTo(new Vector2d(path4dropoff.getX()-48, path4dropoff.getY()-1), 0, slow)
                            .build();
                    drive.followTrajectory(wobble1);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);

                    Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(pickup4.getX() + 24, pickup4.getY(), Math.toRadians(180)))
                            .build();
                    drive.followTrajectory(wobble2);

                    drive.wobble.setPosition(wobbleDown);

                    drive.setPoseEstimate(sensorPoseAnalog());

                    Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeTo(pickup4.vec())
                            .build();
                    drive.followTrajectory(pickupWobble);

                    drive.gripper.setPosition(gripperClosed);
                    sleep(200);
                    drive.wobble.setPosition(wobbleMid);

                    Trajectory drop = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(path4dropoff.getX() - 12, path4dropoff.getY() - 6), omegaGas)
                            .build();
                    drive.followTrajectory(drop);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(150);

                    Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .strafeTo(park4.vec(), omegaGas)
                            .build();
                    drive.followTrajectory(toPark);

                /*currentPose = drive.getPoseEstimate();
                imuHeading = drive.imu.getAngularOrientation().firstAngle;
                drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));

                imuTurn(0);

                System.out.println("Pose: " + drive.getPoseEstimate());

                drive.loader.setPosition(loaded);
                sleep(600);
                drive.loader.setPosition(reload);
                drive.shooter.setVelocity(0);

                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);

                Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(10)
                        .build();
                drive.followTrajectory(park);

                 */
                }
                break;
                case 69: {
                    drive.shooter.setVelocity(-1700);
                    Trajectory go = drive.trajectoryBuilder(startPose)
                            .splineTo(new Vector2d(-1, -47), Math.toRadians(0))
                            .build();
                    drive.followTrajectory(go);

                    drive.setPoseEstimate(new Pose2d(-1, -47, Math.toRadians(180)));
                    System.out.println("Pose: " + drive.getPoseEstimate());

                    drive.loader.setPosition(loaded);
                    sleep(800);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1740);
                    sleep(800);
                    drive.loader.setPosition(loaded);
                    sleep(800);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1720);
                    sleep(800);
                    drive.loader.setPosition(loaded);
                    sleep(800);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(0);
                    drive.ingester.setPower(1);

                    Trajectory move = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(-1, -40), Math.toRadians(90))
                            .build();
                    drive.followTrajectory(move);

                    System.out.println("Pose: " + drive.getPoseEstimate());
                }
            }

            pipeline.initDetect = true;
        } catch (Exception e) {
            System.out.println("Exception: " + e);
        }
    }

    public void imuTurn(double angle) {
        //Radians
        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
        while((Math.abs(imuHeading - angle)>Math.toRadians(2)) && !isStopRequested() && opModeIsActive()){
            System.out.println("Heading check: " + Math.abs(imuHeading - angle));
            System.out.println("Angle: " + angle);
            drive.update();
            imuHeading = drive.imu.getAngularOrientation().firstAngle;
            double tempHeading = imuHeading;
            double tempTarget = angle;
            System.out.println("Heading: " + imuHeading);
            if(tempHeading < 0) tempHeading += 2 * Math.PI;
            if(angle < 0) tempTarget += 2 * Math.PI;
            double p = 0.35, f = 0.05;
            //int invert = tempHeading + (2 * Math.PI - tempTarget) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double invert = angle - imuHeading;
            if(invert > Math.PI) invert -= 2 * Math.PI;
            else if(invert < -Math.PI) invert += 2 * Math.PI;
            invert = invert < 0 ? 1 : -1;
            double power = invert * p * (Math.abs(tempHeading - tempTarget) > Math.PI ? (Math.abs(tempHeading > Math.PI ? 2 * Math.PI - tempHeading : tempHeading) + Math.abs(tempTarget > Math.PI ? 2 * Math.PI - tempTarget : tempTarget)) : Math.abs(tempHeading - tempTarget)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setMotorPowers(power, power, -power, -power);
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public Pose2d sensorPoseAnalog() {
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        //CURRENTLY THE SYSTEM PERPENDICULAR TO WALLS since I would like to test whether the system works before I do fancy stuff.

        //As of April 6, the sensors appear to have a tolerance for 40 degrees either side, with distance not seeming to matter. I would go 30 degrees to be safe.

        //Sensor max is 25 degrees, set to 12.5 degrees on the robot.

        System.out.println("1");

        //drive.imu.gettingInput = true;
        if(isStopRequested()) return null;
        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
        double headingOffsetPlus = imuHeading + Math.toRadians(12.5);
        double headingOffsetMinus = imuHeading - Math.toRadians(12.5);
        double anglePlus = Math.toDegrees(Math.abs(headingOffsetPlus)) % 90;
        anglePlus = anglePlus > 45 ? Math.abs(anglePlus - 90) : anglePlus;
        double angleMinus = Math.toDegrees(Math.abs(headingOffsetMinus)) % 90;
        angleMinus = angleMinus > 45 ? Math.abs(angleMinus - 90) : angleMinus;
        double angleCompensatorPlus = 1 - 0.0002 * anglePlus + 0.0000069 * Math.pow(anglePlus, 2) + 0.00000428 * Math.pow(anglePlus, 3);
        double angleCompensatorMinus = 1 - 0.0002 * angleMinus + 0.0000069 * Math.pow(angleMinus, 2) + 0.00000428 * Math.pow(angleMinus, 3);
        double mult = 86, offset = 0.135;
        double left = mult * (drive.left.getVoltage() - offset);
        double right = mult * (drive.right.getVoltage() - offset);
        double back1 = mult * (drive.back1.getVoltage() - offset);
        double back2 = mult * (drive.back2.getVoltage() - offset);

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

        return new Pose2d(poseX, poseY, imuHeading);
    }
//
//    public Pose2d sensorPose() {
//        //Do not call this in a situation where distance sensors could hit the same wall
//        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).
//
//        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
//        double left = drive.leftDist.getDistance(DistanceUnit.INCH);
//        double right = drive.rightDist.getDistance(DistanceUnit.INCH);
//        double front = drive.frontDist.getDistance(DistanceUnit.INCH);
//        double back = drive.backDist.getDistance(DistanceUnit.INCH);
//        System.out.println("Input : " + left);
//        System.out.println("Right input: " + right);
//        System.out.println("Front input : " + front);
//        System.out.println("Back input: " + back);
//        double correctedSideAngle = sensorSideAngle; //Accounts for X vs. Y.
//        double correctedStraightAngle = sensorStrightAngle; //Accounts for X vs. Y.
//        double correctedHeading = imuHeading > 0 ? (imuHeading + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4 : -((Math.abs(imuHeading) + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4); //Correct heading in each quadrant, as a new quadrant switches what wall it should be seeing.
//        left *= (left < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0; //Gets distance from wall as a straight line
//        System.out.println("Distance : " + left);
//        right *= (right < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier * rightDistMult : 0;
//        front *= (front < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0;
//        back *= (back < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0;
//        left += (left > 0) ? sensorSideOffset * Math.abs(Math.cos(correctedSideAngle + correctedHeading)) : 200; //First quadrant, deals with small cosine values
//        System.out.println("Center Distance : " + left);
//        right += (right > 0) ? sensorSideOffset * Math.abs(Math.cos(-correctedSideAngle + correctedHeading)) : 200; //Second quadrant, deals with small cosine values
//        System.out.println("Right : " + right);
//        front += (front > 0) ? sensorStrightOffset * Math.abs(Math.cos(correctedStraightAngle + correctedHeading)) : 200; //First quadrant, deals with small cosine values
//        System.out.println("Front center : " + front);
//        back += (back > 0) ? sensorStrightOffset * Math.abs(Math.cos(-correctedStraightAngle + correctedHeading)) : 200; //Second quadrant, deals with small cosine values
//        System.out.println("Back center: " + back);
//
//        double distanceYLeft = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? left : front) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? right : back); //Switches which distance sensor input corresponds to what actual side relative to robot.
//        System.out.println("Actual dist : " + distanceYLeft);
//        double distanceYRight = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? right : back) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? left : front);
//        System.out.println("Actual right : " + distanceYRight);
//        double distanceXFront = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? front : right) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? back : left);
//        System.out.println("Actual front : " + distanceXFront);
//        double distanceXBack = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? back : left) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? front : right);
//        System.out.println("Actual back : " + distanceXBack);
//
//        Pose2d currentPose = drive.getPoseEstimate();
//        double poseX = currentPose.getX(), poseY = currentPose.getY();
//
//        poseY = (distanceYRight < distanceYLeft) ? - 87 + Math.abs(distanceYRight) : (distanceYLeft < 100) ? 9 - Math.abs(distanceYLeft) : poseY; //Sets up 0 when robot jammed against left wall, grabs smaller of two distances.
//        poseX = (distanceXBack < distanceXFront) ? - 135 + Math.abs(distanceXBack) : (distanceXFront < 100) ? 9 - Math.abs(distanceXFront) : poseX; //Sets up 0 when robot jammed against front wall
//        System.out.println("Pose Y: " + poseY);
//        System.out.println("Pose X: " + poseX);
//
//        System.out.println("IMU: " + imuHeading);
//
//        return new Pose2d(poseX, poseY, imuHeading);
//    }
//
//    public Pose2d sensorPoseNoBack() {
//        //Do not call this in a situation where distance sensors could hit the same wall
//        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).
//
//        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
//        double left = drive.leftDist.getDistance(DistanceUnit.INCH);
//        double right = drive.rightDist.getDistance(DistanceUnit.INCH);
//        double front = drive.frontDist.getDistance(DistanceUnit.INCH);
//        double back = drive.backDist.getDistance(DistanceUnit.INCH);
//        System.out.println("Input : " + left);
//        System.out.println("Right input: " + right);
//        System.out.println("Front input : " + front);
//        System.out.println("Back input: " + back);
//        double correctedSideAngle = sensorSideAngle; //Accounts for X vs. Y.
//        double correctedStraightAngle = sensorStrightAngle; //Accounts for X vs. Y.
//        double correctedHeading = imuHeading > 0 ? (imuHeading + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4 : -((Math.abs(imuHeading) + Math.PI / 4) % (Math.PI / 2) - Math.PI / 4); //Correct heading in each quadrant, as a new quadrant switches what wall it should be seeing.
//        left *= (left < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0; //Gets distance from wall as a straight line
//        System.out.println("Distance : " + left);
//        right *= (right < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier * rightDistMult : 0;
//        front *= (front < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0;
//        back *= (back < 100) ? Math.abs(Math.cos(correctedHeading)) * multiplier : 0;
//        left += (left > 0) ? sensorSideOffset * Math.abs(Math.cos(correctedSideAngle + correctedHeading)) : 200; //First quadrant, deals with small cosine values
//        System.out.println("Center Distance : " + left);
//        right += (right > 0) ? sensorSideOffset * Math.abs(Math.cos(-correctedSideAngle + correctedHeading)) : 200; //Second quadrant, deals with small cosine values
//        System.out.println("Right : " + right);
//        front += (front > 0) ? sensorStrightOffset * Math.abs(Math.cos(correctedStraightAngle + correctedHeading)) : 200; //First quadrant, deals with small cosine values
//        System.out.println("Front center : " + front);
//        back += (back > 0) ? sensorStrightOffset * Math.abs(Math.cos(-correctedStraightAngle + correctedHeading)) : 200; //Second quadrant, deals with small cosine values
//        System.out.println("Back center: " + back);
//
//        double distanceYLeft = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? left : front) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? right : back); //Switches which distance sensor input corresponds to what actual side relative to robot.
//        System.out.println("Actual dist : " + distanceYLeft);
//        double distanceYRight = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? right : back) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? left : front);
//        System.out.println("Actual right : " + distanceYRight);
//        double distanceXFront = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? front : right) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? back : left);
//        System.out.println("Actual front : " + distanceXFront);
//        double distanceXBack = Math.abs(imuHeading - Math.PI / 4) < Math.PI / 2 ? (Math.abs(imuHeading) < Math.PI / 4 ? back : left) : (Math.abs(imuHeading) > 3 * Math.PI / 4 ? front : right);
//        System.out.println("Actual back : " + distanceXBack);
//
//        Pose2d currentPose = drive.getPoseEstimate();
//        double poseX = currentPose.getX(), poseY = currentPose.getY();
//
//        poseY = (distanceYRight < distanceYLeft) ? - 87 + Math.abs(distanceYRight) : (distanceYLeft < 100) ? 9 - Math.abs(distanceYLeft) : poseY; //Sets up 0 when robot jammed against left wall, grabs smaller of two distances.
//        poseX = (distanceXBack < 100) ? - 135 + Math.abs(distanceXBack) : poseX; //Sets up 0 when robot jammed against front wall
//        System.out.println("Pose Y: " + poseY);
//        System.out.println("Pose X: " + poseX);
//
//        System.out.println("IMU: " + imuHeading);
//
//        return new Pose2d(poseX, poseY, imuHeading);
//    }

    public void turnToPowershot(int pixel) {
        while(Math.abs(minX - pixel) > 3) {
            if(minX == 640) break; //If no value
            double p = 0.0008, f = 0.03;
            double power = p * (minX - pixel);
            power += power > 0 ? f : -f;
            drive.setMotorPowers(power, power, -power, -power);
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public void turnUntilHighGoal() {
        while(Math.abs(upperCameraCenter - targetHighGoalX) > 30) {
            if(upperCameraCenter == 0) break;
            double p = 0.0008, f = 0.03;
            double power = p * (upperCameraCenter - targetHighGoalX);
            power += power > 0 ? f : -f;
            drive.setMotorPowers(power, power, -power, -power);
        }
        drive.setMotorPowers(0, 0, 0, 0);
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

            minX = 640;
            for(int i = 0; i<contours.size()-1; i++){
                if(boundRect[i].tl().x < minX){
                    minX = boundRect[i].tl().x;
                }
            }
            Imgproc.line(outputMat, new Point(minX, 0), new Point(minX, 480),new Scalar(255,0,255), 2);
            Imgproc.putText(outputMat, "Powershot Corner: " + minX, new Point(minX, 100), Imgproc.FONT_HERSHEY_DUPLEX, 0.3, new Scalar(255,255,255));
            //RIGHT POWERSHOT == 70 @ 1600, CENTER == 110 @ 1580, LEFT == 147 @ 1600

            if(contours.size() >= 1) Imgproc.putText(outputMat, "Center: " + (boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x), boundRect[contours.size() - 1].tl(), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255, 255, 255));

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

    static class lowerCameraPipeline extends OpenCvPipeline {

        public double middle = -1, offset = 0;

        public static boolean initDetect = true;

        Mat rawMat = new Mat();
        Mat YCRCBMat = new Mat();
        Mat ExtractMat = new Mat();
        Mat MediumRareMat = new Mat();
        Mat redMat = new Mat();

        enum Stage
        {
            RAW,
            RED,
            EXTRACT,
            MEDIUMRARE
        }

        private lowerCameraPipeline.Stage stageToRenderToViewport = lowerCameraPipeline.Stage.RAW;
        private lowerCameraPipeline.Stage[] stages = lowerCameraPipeline.Stage.values();

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
            if (initDetect) {
                rawMat = input;
                //Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2YCrCb);
                //YCRCBMat = rawMat;
                Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2HSV);
                Core.extractChannel(YCRCBMat, ExtractMat, extract);
                Imgproc.cvtColor(ExtractMat, MediumRareMat, Imgproc.COLOR_GRAY2RGB);

                Core.extractChannel(rawMat, redMat, 0);

                Point topLeft1 = new Point(topCenter.x - sampleWidth,topCenter.y - sampleHeight);
                Point bottomRight1 = new Point(topCenter.x + sampleWidth, topCenter.y + sampleHeight);
                Point topLeft2 = new Point(bottomCenter.x - sampleWidth,bottomCenter.y - sampleHeight);
                Point bottomRight2 = new Point(bottomCenter.x +sampleWidth, bottomCenter.y + sampleHeight);

                color1 = 0;
                color2 = 0;

                for(int i = (int)(topLeft1.x); i <= (int)(bottomRight1.x); i++){
                    for(int j = (int)topLeft1.y;  j <= (int)bottomRight1.y; j++){
                        color1 += ExtractMat.get(j, i)[0];
                    }
                }
                color1 /= (2*sampleWidth + 1)*(2*sampleHeight + 1);

                for(int i = (int)(topLeft2.x); i <= (int)(bottomRight2.x); i++){
                    for(int j = (int)(topLeft2.y);  j <= (int)(bottomRight2.y); j++){
                        color2 += ExtractMat.get(j, i)[0];
                    }
                }
                color2 /= (2*sampleWidth + 1)*(2*sampleHeight + 1);

                boolean yellowness1 = color1 > thresh;
                boolean yellowness2 = color2 > thresh;

                stackSize = yellowness1 ? 4 : yellowness2 ? 1 : 0;

                int numPixels = 0;
                color1 = 0;
                color2 = 0;

                for(int i = (int)(leftBar1.x); i <= (int)(leftBar2.x); i++){
                    for(int j = (int)leftBar1.y;  j <= (int)leftBar2.y; j++){
                        color1 += redMat.get(j, i)[0];
                        numPixels++;
                    }
                }
                color1 /= numPixels;

                numPixels = 0;

                for(int i = (int)(topLeft2.x); i <= (int)(bottomRight2.x); i++){
                    for(int j = (int)(topLeft2.y);  j <= (int)(bottomRight2.y); j++){
                        color2 += redMat.get(j, i)[0];
                        numPixels++;
                    }
                }
                color2 /= numPixels;

                properSetup = (color1 > initThresh) && (color2 > initThresh);

                Imgproc.rectangle(MediumRareMat, topLeft1, bottomRight1, yellowness1 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
                Imgproc.rectangle(MediumRareMat, topLeft2, bottomRight2, yellowness2 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));

                Imgproc.rectangle(MediumRareMat, leftBar1, leftBar2, properSetup ? new Scalar(255, 0, 200) : new Scalar(50, 100, 255));
                Imgproc.rectangle(MediumRareMat, rightBar1, rightBar2, properSetup ? new Scalar(255, 0, 200) : new Scalar(50, 100, 255));

                Core.flip(MediumRareMat, MediumRareMat, -1);
                Core.flip(redMat, redMat, -1);
            }
            else{
                rawMat = input;
                //Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2YCrCb);
                //YCRCBMat = rawMat;
                Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2HSV);
                Core.extractChannel(YCRCBMat, ExtractMat, extract);
                Imgproc.cvtColor(ExtractMat, MediumRareMat, Imgproc.COLOR_GRAY2RGB);
                Imgproc.line(MediumRareMat, new Point(0, row), new Point(640, row), new Scalar(255,0,0), 3);
                double wobbleLeft = -1, wobbleRight = -1;
                for(int x = 0; x < MediumRareMat.cols(); x++){
                    int counter = 0;
                    double[] pixel = ExtractMat.get(row,x);
                    if(pixel[0]>wobbleThresh){
                        Imgproc.line(MediumRareMat, new Point(x, 300), new Point(x, 340), new Scalar(0,255,0), 3);
                        if((x<630 && x>10) && (ExtractMat.get(row, x-8)[0]>wobbleThresh) && (ExtractMat.get(row, x+8)[0]>wobbleThresh)){
                            if(wobbleLeft == -1) wobbleLeft = x;
                            wobbleRight = x;
                            Imgproc.line(MediumRareMat, new Point(x, 0), new Point(x, 480), new Scalar(0,0,255), 5);
                        }
                    }
                }
                if(wobbleLeft != wobbleRight) {
                    middle = (wobbleLeft + wobbleRight) / 2.0;
                    offset = (320 - middle) / offsetDivisor - 2.5;
                }
            }
            switch (stageToRenderToViewport){
                case RAW:
                {
                    return MediumRareMat;
                }
                case EXTRACT:
                {
                    return ExtractMat;
                }
                case RED:
                {
                    return redMat;
                }
                default:
                {
                    return input;
                }
            }
        }
    }
}
