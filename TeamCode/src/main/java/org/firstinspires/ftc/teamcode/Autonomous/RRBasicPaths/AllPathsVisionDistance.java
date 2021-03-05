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

@Config
@Autonomous
public class AllPathsVisionDistance extends LinearOpMode {

    private DriveConstraints verySlow = new DriveConstraints(
            10, 15.0, 0.0,
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
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0
    );

    private Pose2d startPose = new Pose2d(-126.0, -50.0, Math.toRadians(0.0)); //-18.0
    private Pose2d midwayShoot = new Pose2d(-101.0, -33.0);
    private Pose2d powerShotShoot = new Pose2d(-87.0, -39.0);
    private Pose2d highGoalShoot = new Pose2d(-82.0, -60.0);
    private Pose2d path4dropoff = new Pose2d(-20.0, -69.0);
    private Pose2d pickup4 = new Pose2d(-120, -68.5, 0.0);
    private Pose2d path1dropoff = new Pose2d(-48.0, -54.0);
    private Pose2d pickup1 = new Pose2d(-121, -68.5); //Was x = -38.5, y = -40 before moving to back of tape
    private Pose2d path0dropoff = new Pose2d(-65.0, -71.0);
    private Pose2d pickup0 = new Pose2d(-120, -68.5);
    private Pose2d endLocation = new Pose2d(-58.0, -59.0, 0.0);

    public static double wobbleUp = 0.17, wobbleDown = 0.65, wobbleMid = 0.45, wobblePushStack = 0.57, gripperOpen = 0, gripperClosed = 1, loaded = 0.48, reload = 0.14, path5highgoalX = -80, path5highgoalY = -60, offsetDivisor = 50, pshot1 = 5, pshot2 = 4.7, pshot3 = 8.3;

    public static double multiplier = 0.97, sensorSideOffset = 8.20, sensorSideAngle = 0.66, sensorStrightOffset = 8, sensorStrightAngle = 0, rightDistMult = 1;

    private Pose2d powerShotBackShoot = new Pose2d(-102.0,-50.0, Math.toRadians(pshot1)); //Is actually 1
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

            DistanceSensor leSense = hardwareMap.get(DistanceSensor.class, "distanceRight");

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

            drive.wobble.setPosition(wobbleUp);

            while (!isStarted() && !isStopRequested()) {
                telemetry.addData("Le stack: ", stackSize);
                telemetry.addData("Setup: ", properSetup);
                telemetry.addData("Distance from wall: ", leSense.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }

            if (isStopRequested()) return;

            int stackSize2 = stackSize;

            drive.setPoseEstimate(startPose);

            drive.shooter.setVelocity(-1640);

            drive.wobble.setPosition(wobbleMid);

            Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                    .strafeTo(new Vector2d(powerShotBackShoot.getX()-6, powerShotBackShoot.getY()+5))
                    .build();
            drive.followTrajectory(goToShoot);

            stackSize2 = 4;
//
//            turnToPowershot(23); //Does not see far left powershot yet, the turn to 3 degrees moves the left powershot in view.
//
//            drive.loader.setPosition(loaded);
//            sleep(600);
//            drive.loader.setPosition(reload);
//            drive.shooter.setVelocity(-1580);
//            imuTurn(Math.toRadians(3));
//            //imuTurn(Math.toRadians(pshot2));
//            turnToPowershot(30);
//            sleep(400);
//            drive.loader.setPosition(loaded);
//            sleep(600);
//            drive.loader.setPosition(reload);
//            drive.shooter.setVelocity(-1600);
//            //imuTurn(Math.toRadians(pshot3));
//            turnToPowershot(70);
//            sleep(400);
//            drive.loader.setPosition(loaded);
//            sleep(600);
//            drive.shooter.setVelocity(0);

            double imuHeading;// = drive.imu.getAngularOrientation().firstAngle;
            Pose2d currentPose = sensorPose();

            switch (stackSize2) {
                case 0: {

                    drive.ingester.setPower(0);

                    //drive.setPoseEstimate(currentPose);

                    imuTurn(0);
                    drive.setPoseEstimate(sensorPose());
                    System.out.println("Sensor Pose: " + sensorPose());
                    System.out.println("Current Pose: " + drive.getPoseEstimate());
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));

                    Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeTo(new Vector2d(path0dropoff.getX(), path0dropoff.getY()), slow)
                            .build();
                    drive.followTrajectory(wobble1);

                    drive.loader.setPosition(reload);

                    //currentPose = drive.getPoseEstimate();
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
                    //drive.setPoseEstimate(sensorPose());

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1200);
                    drive.gripper.setPosition(gripperClosed);
                    drive.wobble.setPosition(wobbleUp);

                    drive.setPoseEstimate(sensorPose());

                    Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .lineToLinearHeading(new Pose2d(pickup0.getX() + 24, pickup0.getY() + 2, Math.toRadians(180.0)), verySlow)
                            .build();
                    drive.followTrajectory(wobble2);

                    pipeline.initDetect = false;

                    double offset = pipeline.offset;

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(600);

                    drive.setPoseEstimate(sensorPose());
                    //currentPose = drive.getPoseEstimate();
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() - offset - 1, imuHeading));

                    Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(pickup0.getX() + 6, pickup0.getY()), Math.toRadians(180), verySlow)
                            .splineToConstantHeading(new Vector2d(pickup0.getX(), pickup0.getY()), Math.toRadians(180), verySlow)
                            .build();
                    drive.followTrajectory(pickupWobble);

                    //currentPose = drive.getPoseEstimate();
                    //imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() + offset + 1, imuHeading));
                    drive.setPoseEstimate(sensorPose()); //Should account for offset, so no need for previous line.

                    drive.gripper.setPosition(gripperClosed);
                    sleep(1200);
                    drive.wobble.setPosition(0.6);

                    Trajectory dropoff = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(path0dropoff.getX() - 7, path0dropoff.getY() + 6, 0.0), slow)
                            .build();
                    drive.followTrajectory(dropoff);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1500);

                    drive.wobble.setPosition(wobbleUp);
                    sleep(500);

                    drive.setPoseEstimate(sensorPose());

                    Trajectory park1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeLeft(24)
                            .build();
                    drive.followTrajectory(park1);

                    Trajectory park2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(18)
                            .build();
                    drive.followTrajectory(park2);
                }
                break;
                case 1: {

                    drive.ingester.setPower(1);
                    drive.preIngest.setPower(1);

                    drive.setPoseEstimate(currentPose);

                    Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(path1dropoff.vec(), 0)
                            .build();
                    drive.followTrajectory(wobble1);

                    drive.loader.setPosition(reload);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1200);

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    drive.setPoseEstimate(sensorPose());

                    Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .lineToLinearHeading(new Pose2d(pickup1.getX() + 24, pickup1.getY(), Math.toRadians(180.0)), slow)
                            .build();
                    drive.followTrajectory(wobble2);

                    pipeline.initDetect = false;

                    sleep(300);

                    double offset = pipeline.offset;

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() - offset + 1, currentPose.getHeading()));
                    drive.setPoseEstimate(sensorPose());

                    drive.ingester.setPower(0);
                    drive.preIngest.setPower(0);

                    Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(pickup1.getX() + 6, pickup1.getY()), Math.toRadians(180), verySlow)
                            .splineToConstantHeading(new Vector2d(pickup1.getX(), pickup1.getY()), Math.toRadians(180), verySlow)
                            .build();
                    drive.followTrajectory(pickupWobble);

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() + offset - 1, imuHeading));
                    drive.setPoseEstimate(sensorPose());

                    drive.gripper.setPosition(gripperClosed);
                    sleep(1000);
                    drive.wobble.setPosition(0.55);
                    drive.shooter.setVelocity(-1640);

                    Trajectory shoot = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                            .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), Math.toRadians(0.0)), slow)
                            .build();
                    drive.followTrajectory(shoot);

                    //Maybe add high goal turn

                    drive.ingester.setPower(0);
                    turnUntilHighGoal();
                    //drive.wobble.setPosition(wobbleMid);

                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.loader.setPosition(reload);

                    Trajectory drop = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                            .splineTo(new Vector2d(path1dropoff.getX(), path1dropoff.getY() + 8), Math.toRadians(0.0), kindaFast)
                            .build();
                    drive.followTrajectory(drop);

                    drive.shooter.setVelocity(0);
                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(1200);
                    drive.wobble.setPosition(wobbleUp);

                    Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                            .back(10, omegaFast)
                            .build();
                    drive.followTrajectory(toPark);
                }
                break;
                case 4: {
                    drive.ingester.setPower(1);
                    drive.preIngest.setPower(-1);
                    //drive.wobble.setPosition(wobbleMid);

                    imuTurn(0);

                    drive.wobble.setPosition(wobblePushStack);

                    Trajectory pushRings2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(powerShotBackShoot.getX()+4, powerShotBackShoot.getY()+6), 0, slow)
                            .addDisplacementMarker(() -> {
                                drive.wobble.setPosition(wobbleUp);
                                drive.shooter.setVelocity(-1640);
                            })
                            .splineToConstantHeading(new Vector2d(powerShotBackShoot.getX()+2, powerShotBackShoot.getY()-1.5), 0, verySlow)
                            .splineToConstantHeading(new Vector2d(powerShotBackShoot.getX()+19, powerShotBackShoot.getY()-0.5), 0, verySlow)
                            //.splineTo(new Vector2d(path4dropoff.getX()-48, path4dropoff.getY()-1), 0, slow)
                            .build();
                    drive.followTrajectory(pushRings2);

                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.loader.setPosition(reload);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.loader.setPosition(reload);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.loader.setPosition(reload);
                    sleep(400);

                    Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(powerShotBackShoot.getX() + 48, powerShotBackShoot.getY()), Math.toRadians(0))
                            .splineToConstantHeading(new Vector2d(path4dropoff.getX(), path4dropoff.getY()), Math.toRadians(0))
                            .build();
                    drive.followTrajectory(wobble1);

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    if (imuHeading < 0) imuHeading += 2 * Math.PI;
                    System.out.println("IMU: " + imuHeading);
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
                    drive.setPoseEstimate(sensorPose());

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(850); //Extra 500 to let it continue ingesting

                    Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .lineToLinearHeading(new Pose2d(pickup4.getX() + 24, pickup4.getY() + 4, Math.toRadians(180.0)))
                            .build();
                    drive.followTrajectory(wobble2);

                    pipeline.initDetect = false;

                    imuTurn(Math.PI);

                    double offset = pipeline.offset;

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    System.out.println("IMU: " + imuHeading + ", Odo: " + currentPose.getHeading());
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() - offset - 4, currentPose.getHeading()));
                    drive.setPoseEstimate(sensorPose());

                    Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineToConstantHeading(new Vector2d(pickup4.getX() + 6, pickup4.getY()), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(pickup4.getX(), pickup4.getY()), Math.toRadians(180))
                            .build();
                    drive.followTrajectory(pickupWobble);

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY() + offset + 4, imuHeading));
                    drive.setPoseEstimate(sensorPose());

                    drive.gripper.setPosition(gripperClosed);
                    sleep(1200);
                    drive.wobble.setPosition(wobbleUp);
                    drive.shooter.setVelocity(-1660);
                    drive.loader.setPosition(reload);
                    sleep(200);

                    Trajectory shootdrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(path5highgoalX, path5highgoalY, Math.toRadians(0)))
                            .build();
                    drive.followTrajectory(shootdrop);

                    drive.wobble.setPosition(wobbleMid);

                    sleep(500);

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
                    drive.setPoseEstimate(sensorPose());

                    System.out.println("Pose: " + drive.getPoseEstimate());

                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.loader.setPosition(reload);
                    drive.shooter.setVelocity(-1600);
                    sleep(400);
                    drive.loader.setPosition(loaded);
                    sleep(600);
                    drive.shooter.setVelocity(0);

                    Trajectory drop = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .splineTo(new Vector2d(path4dropoff.getX() - 36, path4dropoff.getY() + 16), 0, omegaFast)
                            .splineTo(new Vector2d(path4dropoff.getX() - 24, path4dropoff.getY() + 16), 0, omegaFast)
                            .splineTo(new Vector2d(path4dropoff.getX() - 6, path4dropoff.getY() + 6), 0, omegaFast)
                            .build();
                    drive.followTrajectory(drop);

                    drive.wobble.setPosition(wobbleDown);
                    drive.gripper.setPosition(gripperOpen);
                    sleep(850);
                    drive.shooter.setVelocity(-1720);
                    drive.loader.setPosition(reload);

                    currentPose = drive.getPoseEstimate();
                    imuHeading = drive.imu.getAngularOrientation().firstAngle;
                    //drive.setPoseEstimate(new Pose2d(currentPose.getX(), currentPose.getY(), imuHeading));
                    drive.setPoseEstimate(sensorPose());

                    System.out.println("Pose: " + drive.getPoseEstimate());

                    Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                            .back(10, omegaFast)
                            .addDisplacementMarker(() -> {
                                drive.gripper.setPosition(gripperClosed);
                                drive.wobble.setPosition(wobbleUp);
                            })
                            .back(30, omegaFast)
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

        double imuHeading = drive.imu.getAngularOrientation().firstAngle;

        while((Math.abs(imuHeading - angle)>Math.toRadians(0.1)) && !isStopRequested() && opModeIsActive()){
            drive.update();
            imuHeading = drive.imu.getAngularOrientation().firstAngle;
            if(imuHeading < 0) imuHeading += 2 * Math.PI;
            //currentPose = drive.getPoseEstimate();
            double p = 0.2, f = 0.05;
            int invert = (angle + (2 * Math.PI - imuHeading)) % (2 * Math.PI) > Math.PI ? 1 : -1;
            double power = invert * p * (Math.abs(imuHeading - angle) > Math.PI ? (Math.abs((imuHeading > Math.PI ? 2 * Math.PI : 0) - imuHeading) + Math.abs((angle > Math.PI ? 2 * Math.PI : 0) - angle)) : Math.abs(imuHeading - angle)); //Long line, but the gist is if you're calculating speed in the wrong direction, git gud.
            power += (power > 0 ? f : -f);
            drive.setMotorPowers(power, power, -power, -power);

            telemetry.addData("IMU Heading: ", imuHeading);
            telemetry.addData("Power: ", power);
            telemetry.addData("Invert: ", invert);
            telemetry.addData("Invert calc: ", (angle + (360 - imuHeading)));
            telemetry.update();
        }
        drive.setMotorPowers(0, 0, 0, 0);
    }

    public Pose2d sensorPose() {
        //Do not call this in a situation where distance sensors could hit the same wall
        //NOTE: DO NOT call repeatedly (aka every loop) or the y position will not update properly (while using odo).

        double imuHeading = drive.imu.getAngularOrientation().firstAngle;
        double left = drive.leftDist.getDistance(DistanceUnit.INCH);
        double right = drive.rightDist.getDistance(DistanceUnit.INCH);
        double front = drive.frontDist.getDistance(DistanceUnit.INCH);
        double back = drive.backDist.getDistance(DistanceUnit.INCH);
        System.out.println("Input : " + left);
        System.out.println("Right input: " + right);
        System.out.println("Front input : " + front);
        System.out.println("Back input: " + back);
        double correctedSideAngle = sensorSideAngle; //Accounts for X vs. Y.
        double correctedStraightAngle = sensorStrightAngle; //Accounts for X vs. Y.
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

        Pose2d currentPose = drive.getPoseEstimate();
        double poseX = currentPose.getX(), poseY = currentPose.getY();

        poseY = (distanceYRight < distanceYLeft) ? - 87 + Math.abs(distanceYRight) : (distanceYLeft < 100) ? 9 - Math.abs(distanceYLeft) : poseY; //Sets up 0 when robot jammed against left wall, grabs smaller of two distances.
        poseX = (distanceXBack < distanceXFront) ? - 135 + Math.abs(distanceXBack) : (distanceXFront < 100) ? 9 - Math.abs(distanceXFront) : poseX; //Sets up 0 when robot jammed against front wall
        System.out.println("Pose Y: " + poseY);
        System.out.println("Pose X: " + poseX);

        System.out.println("IMU: " + imuHeading);

        return new Pose2d(poseX, poseY, imuHeading);
    }

    public void turnToPowershot(int pixel) {
        while(Math.abs(minX - pixel) > 2) {
            if(minX == 640) break; //If no value
            double p = 0.0008, f = 0.025;
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
