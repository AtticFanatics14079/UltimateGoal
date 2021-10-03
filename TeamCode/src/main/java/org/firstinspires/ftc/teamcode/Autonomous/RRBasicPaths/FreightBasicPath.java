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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.leftUL;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.middleUL;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.rightUL;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.sampleHeight;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.sampleWidth;
import static org.firstinspires.ftc.teamcode.Vision.BarCodeDuckPipeline.thresh;

/*
 * Op mode for tuning follower PID coefficients (located in the drive base classes). The robot
 * drives in a DISTANCE-by-DISTANCE square indefinitely.
 */
@Config
@Autonomous(group = "drive")
public class FreightBasicPath extends LinearOpMode {
    private Pose2d startPose = new Pose2d(72.0, 36.0, Math.toRadians(180));
    private Pose2d spinPose = new Pose2d(67.0 , 63.0, Math.toRadians(270));
    private Pose2d allianceGoalPose = new Pose2d(24.0, 12.0, Math.toRadians(0));
    private Pose2d allianceGoalDropOffPose = new Pose2d(24.0, 22.0, Math.toRadians(100));
    private Pose2d tapedParkPose = new Pose2d(36.0,60.0,90);

    public static int duckLocation = -1;

    public static double level1 = -2500, level2 = -5000;

    public static double OPEN = 0, CLOSED = 0, back1 = 8, forward1 = 24, back2 = 48, forward2 = 20, strafe = 54;

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive.setPoseEstimate(startPose);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new duckScanPipeline());
        webCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);//display on RC

        while(!isStarted() && !isStopRequested()) {
            drive.update();
            telemetry.addData("Position: ", drive.getPoseEstimate());
            telemetry.update();
        }

        waitForStart();
        if(isStopRequested()) return;

        double slideTicks = 0;
        if(duckLocation > 0) slideTicks = duckLocation == 1 ? level1 : level2;

        System.out.println("Duck: " + duckLocation + ", ticks: " + slideTicks);

        telemetry.addData("Position: ", drive.getPoseEstimate());
        telemetry.update();

        Trajectory back = drive.trajectoryBuilder(startPose)
                .back(back1)
                .build();
        drive.followTrajectory(back);

        drive.turn(Math.toRadians(90));

        Trajectory goToSpin = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(forward1)
                .build();
        drive.followTrajectory(goToSpin);

        /*Trajectory touchSpin = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(3, new DriveConstraints(20, 10, 0.0, Math.toRadians(140.0), Math.toRadians(140.0), 0.0))
                .build();
        drive.followTrajectory(touchSpin);

         */

        drive.spinner.setPower(-0.4);
        sleep(3500);
        drive.spinner.setPower(0);

        /*Trajectory goToDropOff = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(new Vector2d(allianceGoalDropOffPose.getX(), allianceGoalDropOffPose.getY()), 0)
                .build();
        drive.followTrajectory(goToDropOff);

         */

        Trajectory goToDropOff = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(back2)
                .build();
        drive.followTrajectory(goToDropOff);

        imuTurn(Math.toRadians(0));

        Trajectory goToDropOff2 = drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(forward2)
                .build();
        drive.followTrajectory(goToDropOff2);

        while(Math.abs(drive.slides.getCurrentPosition() - slideTicks) > 50) {
            drive.slides.setPower(-0.8);
        }
        drive.slides.setPower(0);

        drive.dropper.setPosition(OPEN);
        sleep(600);

        Trajectory chimichanga = drive.trajectoryBuilder(drive.getPoseEstimate())
                //.splineTo(tapedParkPose.vec(), 0)
                .back(-2)
                .build();
        drive.followTrajectory(chimichanga);

        Trajectory shimishanga = drive.trajectoryBuilder(drive.getPoseEstimate())
                //.splineTo(tapedParkPose.vec(), 0)
                .forward(-2)
                .build();
        drive.followTrajectory(shimishanga);

        Trajectory toPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                //.splineTo(tapedParkPose.vec(), 0)
                .strafeRight(strafe)
                .build();
        drive.followTrajectory(toPark);
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
            double p = 0.3, f = 0.04;
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

    static class duckScanPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat YCRCBMat = new Mat();
        Mat ExtractMat = new Mat();

        enum Stage
        {
            RAW,
            EXTRACT,
            RED
        }

        private Stage stageToRenderToViewport = Stage.EXTRACT;
        private Stage[] stages = Stage.values();

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
            rawMat = input;
            //Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2YCrCb);
            //YCRCBMat = rawMat;
            //Mat rota = Imgproc.getRotationMatrix2D(new Point(160, 120), rotateAngle,1);
            //Imgproc.warpAffine(rawMat, rawMat, rota, new Size(320,240));
            Imgproc.cvtColor(rawMat, YCRCBMat, 2);
            Core.extractChannel(YCRCBMat, ExtractMat, 1);

            double color1 = 0, color2 = 0, color3 = 0;

            for(int i = (int)(leftUL.x); i <= (int)(leftUL.x + sampleWidth); i++){
                for(int j = (int)(leftUL.y);  j <= (int)(leftUL.y + sampleHeight); j++){
                    color1 += ExtractMat.get(j, i)[0];
                }
            }
            color1 /= sampleWidth * sampleHeight;

            for(int i = (int)(middleUL.x); i <= (int)(middleUL.x + sampleWidth); i++){
                for(int j = (int)(middleUL.y);  j <= (int)(middleUL.y + sampleHeight); j++){
                    color2 += ExtractMat.get(j, i)[0];
                }
            }
            color2 /= sampleWidth * sampleHeight;

            for(int i = (int)(rightUL.x); i <= (int)(rightUL.x + sampleWidth); i++){
                for(int j = (int)(rightUL.y);  j <= (int)(rightUL.y + sampleHeight); j++){
                    color3 += ExtractMat.get(j, i)[0];
                }
            }
            color3 /= sampleWidth * sampleHeight;

            boolean leftDuck = color1 > thresh;
            boolean midDuck = color2 > thresh;
            boolean rightDuck = color3 > thresh;

            System.out.println("Color1: " + color1 + ", Color2: " + color2 + "Color3: " + color3);

            if(leftDuck && !midDuck && !rightDuck) duckLocation = 0;
            else if(!leftDuck && midDuck && !rightDuck) duckLocation = 1;
            else if(!leftDuck && !midDuck && rightDuck) duckLocation = 2;
            else duckLocation = -1;

            Imgproc.rectangle(ExtractMat, leftUL, new Point(leftUL.x + sampleWidth, leftUL.y + sampleHeight), leftDuck ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            Imgproc.rectangle(ExtractMat, middleUL, new Point(middleUL.x + sampleWidth, middleUL.y + sampleHeight), midDuck ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            Imgproc.rectangle(ExtractMat, rightUL, new Point(rightUL.x + sampleWidth, rightUL.y + sampleHeight), rightDuck ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));

            switch (stageToRenderToViewport){
                case RAW:
                {
                    return rawMat;
                }
                case EXTRACT:
                {
                    return ExtractMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}

