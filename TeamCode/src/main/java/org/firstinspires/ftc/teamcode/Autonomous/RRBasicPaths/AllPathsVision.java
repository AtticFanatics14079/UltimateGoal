package org.firstinspires.ftc.teamcode.Autonomous.RRBasicPaths;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.scanPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
@Autonomous
public class AllPathsVision extends LinearOpMode {

    private DriveConstraints constraints = new DriveConstraints(45.0, 30.0, 0.0, Math.toRadians(270), Math.toRadians(270), 0.0);
    private Pose2d startPose = new Pose2d(-63.0, -33.0, Math.toRadians(0.0)); //-18.0
    private Pose2d midwayShoot = new Pose2d(-38.0, -18.0);
    private Pose2d powerShotShoot = new Pose2d(-24.0, -24.0);
    private Pose2d highGoalShoot = new Pose2d(-12.0, -40.0);
    private Pose2d path4dropoff = new Pose2d(43.0, -44.0);
    private Pose2d pickup4 = new Pose2d(-53.0, -52, 0.0);
    private Pose2d path1dropoff = new Pose2d(15.0, -34.0);
    private Pose2d pickup1 = new Pose2d(-53, -55.25); //Was x = -38.5, y = -40 before moving to back of tape
    private Pose2d path0dropoff = new Pose2d(-0.0, -60.0);
    private Pose2d pickup0 = new Pose2d(-56.5, -50.0);
    private Pose2d endLocation = new Pose2d(5.0, -44.0, 0.0);

    public static double wobbleUp = 0.22, wobbleDown = 0.65, wobbleMid = 0.45, gripperOpen = 0, gripperClosed = 1;

    private final int rows = 640;
    private final int cols = 480;
    public static int sampleWidth = 3;
    public static int sampleHeight = 2;
    public static Point topCenter = new Point(510, 420);
    public static Point bottomCenter = new Point(510, 350);
    public static int thresh = 155;
    public static int stackSize = 4; //For testing, choose whatever path + disable camera
    public static boolean usingCamera = false;
    private static double color1, color2;
    OpenCvCamera webCam;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if(usingCamera) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
            webCam.openCameraDevice();//open camera
            webCam.setPipeline(new StageSwitchingPipeline());//different stages
            webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        }

        drive.loader.setPosition(0.25);

        drive.wobble.setPosition(wobbleDown);

        drive.gripper.setPosition(gripperClosed);

        sleep(1200);

        drive.wobble.setPosition(wobbleMid);

        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Le stack: ", stackSize);
            telemetry.update();
        }

        if(isStopRequested()) return;

        drive.setPoseEstimate(startPose);

        drive.shooter.setVelocity(-1560);

        switch(stackSize) {
            case 0: {
                Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                        //.splineTo(midwayShoot.vec(), Math.toRadians(5))
                        .splineTo(powerShotShoot.vec(), Math.toRadians(-3))
                        .build();
                drive.followTrajectory(goToShoot);

                drive.loader.setPosition(0.6);
                sleep(800);
                drive.shooter.setVelocity(-1560);
                drive.loader.setPosition(0.15);
                drive.turn(Math.toRadians(4));
                sleep(200);
                drive.loader.setPosition(0.6);
                sleep(800);
                drive.shooter.setVelocity(-1560);
                drive.loader.setPosition(0.15);
                drive.turn(Math.toRadians(4));
                sleep(200);
                drive.loader.setPosition(0.6);
                sleep(800);
                drive.loader.setPosition(0.15);
                drive.shooter.setVelocity(0);

                Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(path0dropoff.getX(), path0dropoff.getY(), Math.toRadians(0)))
                        .build();
                drive.followTrajectory(wobble1);

                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);
                sleep(1000);

                Trajectory toWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(pickup0.getX() + 12, pickup0.getY(), Math.toRadians(180.0)))
                        .build();
                drive.followTrajectory(toWobble);

                Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .forward(10)
                        .build();
                drive.followTrajectory(pickupWobble);

                drive.gripper.setPosition(gripperClosed);
                sleep(800);
                drive.wobble.setPosition(wobbleMid);

                Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(endLocation.getX(), endLocation.getY() - 8, 0.0))
                        .build();
                drive.followTrajectory(wobble2);

                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);
                sleep(1500);
                drive.wobble.setPosition(wobbleMid);

                Trajectory toEnd = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(endLocation)
                        .build();
                drive.followTrajectory(toEnd);
            }
            break;
            case 1: {
                Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                        //.splineTo(midwayShoot.vec(), Math.toRadians(5))
                        .splineTo(powerShotShoot.vec(), Math.toRadians(-3))
                        .build();
                drive.followTrajectory(goToShoot);

                drive.loader.setPosition(0.6);
                sleep(800);
                drive.shooter.setVelocity(-1560);
                drive.loader.setPosition(0.15);
                drive.turn(Math.toRadians(4));
                sleep(200);
                drive.loader.setPosition(0.6);
                sleep(800);
                drive.shooter.setVelocity(-1560);
                drive.loader.setPosition(0.15);
                drive.turn(Math.toRadians(4));
                sleep(200);
                drive.loader.setPosition(0.6);
                sleep(800);
                drive.loader.setPosition(0.15);
                drive.shooter.setVelocity(0);

                Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(path1dropoff.vec(), 0)
                        .build();
                drive.followTrajectory(wobble1);

                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);
                sleep(1000);

                drive.ingester.setPower(1);

                Trajectory rings = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .lineToLinearHeading(new Pose2d(highGoalShoot.getX() + 4, highGoalShoot.getY() + 6, Math.toRadians(180)))
                        //.splineTo(new Vector2d(highGoalShoot.getX() + 4, highGoalShoot.getY()), Math.toRadians(180.0))
                        .build();
                drive.followTrajectory(rings);

                Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .addDisplacementMarker(() -> drive.ingester.setPower(1))
                        .splineTo(new Vector2d(pickup1.getX(), pickup1.getY()), Math.toRadians(180)) //Was Math.toRadians(215) before moving to back of tape
                        .build();
                drive.followTrajectory(wobble2);

                drive.gripper.setPosition(gripperClosed);
                sleep(800);
                drive.wobble.setPosition(wobbleMid);
                drive.shooter.setVelocity(-1700);

                Trajectory shoot = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .lineToLinearHeading(new Pose2d(highGoalShoot.getX(), highGoalShoot.getY(), Math.toRadians(0.0)))
                        .build();
                drive.followTrajectory(shoot);

                drive.ingester.setPower(0);

                drive.loader.setPosition(0.6);
                sleep(800);
                drive.loader.setPosition(0.15);

                Trajectory drop = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .splineTo(new Vector2d(path1dropoff.getX(), path1dropoff.getY() + 10), Math.toRadians(0.0))
                        .build();
                drive.followTrajectory(drop);

                drive.shooter.setVelocity(0);
                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);
                sleep(1000);

                Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                        .lineToLinearHeading(endLocation)
                        .build();
                drive.followTrajectory(toPark);
            }
            break;
            case 4: {
                Trajectory goToShoot = drive.trajectoryBuilder(startPose)
                        //.splineTo(midwayShoot.vec(), Math.toRadians(5))
                        .splineTo(powerShotShoot.vec(), Math.toRadians(-3))
                        .build();
                drive.followTrajectory(goToShoot);

                drive.loader.setPosition(0.6);
                sleep(800);
                drive.shooter.setVelocity(-1560);
                drive.loader.setPosition(0.15);
                drive.turn(Math.toRadians(4));
                sleep(200);
                drive.loader.setPosition(0.6);
                sleep(800);
                drive.shooter.setVelocity(-1560);
                drive.loader.setPosition(0.15);
                drive.turn(Math.toRadians(4));
                sleep(200);
                drive.loader.setPosition(0.6);
                sleep(800);
                drive.loader.setPosition(0.15);
                drive.shooter.setVelocity(0);

                Trajectory wobble1 = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(path4dropoff.getX(), path4dropoff.getY() - 6), Math.toRadians(0.0))
                        .build();
                drive.followTrajectory(wobble1);

                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);
                sleep(1000);

                Trajectory wobble2 = drive.trajectoryBuilder(drive.getPoseEstimate(), true)
                        .lineToLinearHeading(new Pose2d(pickup4.getX() + 24, pickup4.getY(), Math.toRadians(180.0)))
                        .build();
                drive.followTrajectory(wobble2);

                Trajectory pickupWobble = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeTo(new Vector2d(pickup4.getX(), pickup4.getY()))
                        .build();
                drive.followTrajectory(pickupWobble);

                drive.gripper.setPosition(gripperClosed);
                sleep(800);
                drive.wobble.setPosition(wobbleMid);
                //drive.shooter.setVelocity(-1700);

                Trajectory shootdrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(highGoalShoot.getX() -22, highGoalShoot.getY() - 4, 0))
                        .build();
                drive.followTrajectory(shootdrop);

                Trajectory drop = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(new Vector2d(path4dropoff.getX(), path4dropoff.getY() + 6), 0.0)
                        .build();
                drive.followTrajectory(drop);

                drive.wobble.setPosition(wobbleDown);
                drive.gripper.setPosition(gripperOpen);
                sleep(1000);

                Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .strafeTo(new Vector2d(10.0, path4dropoff.getY()))
                        .build();
                drive.followTrajectory(toPark);

            }
        }
        drive.gripper.setPosition(gripperClosed);
        sleep(500);
        drive.wobble.setPosition(wobbleUp);
        sleep(800);
    }

    static class StageSwitchingPipeline extends OpenCvPipeline {
        Mat rawMat = new Mat();
        Mat YCRCBMat = new Mat();
        Mat ExtractMat = new Mat();
        Mat MediumRareMat = new Mat();

        enum Stage {
            RAW,
            YCRCB,
            EXTRACT,
            MEDIUMRARE
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.RAW;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

        @Override
        public void onViewportTapped() {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if (nextStageNum >= stages.length) {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input) {
            rawMat = input;
            Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2HSV);
            Core.extractChannel(YCRCBMat, ExtractMat, 1);
            Imgproc.cvtColor(ExtractMat, MediumRareMat, Imgproc.COLOR_GRAY2RGB);

            Point topLeft1 = new Point(topCenter.x - sampleWidth, topCenter.y - sampleHeight);
            Point bottomRight1 = new Point(topCenter.x + sampleWidth, topCenter.y + sampleHeight);
            Point topLeft2 = new Point(bottomCenter.x - sampleWidth, bottomCenter.y - sampleHeight);
            Point bottomRight2 = new Point(bottomCenter.x + sampleWidth, bottomCenter.y + sampleHeight);

            color1 = 0;
            color2 = 0;

            for (int i = (int) (topLeft1.x); i <= (int) (bottomRight1.x); i++) {
                for (int j = (int) topLeft1.y; j <= (int) bottomRight1.y; j++) {
                    color1 += ExtractMat.get(j, i)[0];
                }
            }
            color1 /= (2 * sampleWidth + 1) * (2 * sampleHeight + 1);

            for (int i = (int) (topLeft2.x); i <= (int) (bottomRight2.x); i++) {
                for (int j = (int) (topLeft2.y); j <= (int) (bottomRight2.y); j++) {
                    color2 += ExtractMat.get(j, i)[0];
                }
            }
            color2 /= (2 * sampleWidth + 1) * (2 * sampleHeight + 1);

            boolean yellowness1 = color1 > thresh;
            boolean yellowness2 = color2 > thresh;

            stackSize = yellowness1 ? 4 : yellowness2 ? 1 : 0;

            Imgproc.rectangle(MediumRareMat, topLeft1, bottomRight1, yellowness1 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            Imgproc.rectangle(MediumRareMat, topLeft2, bottomRight2, yellowness2 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));

            switch (stageToRenderToViewport) {
                case RAW: {
                    return MediumRareMat;
                }
                case YCRCB: {
                    return YCRCBMat;
                }
                case EXTRACT: {
                    return ExtractMat;
                }
                case MEDIUMRARE: {
                    return MediumRareMat;
                }
                default: {
                    return input;
                }
            }
        }
    }
}
