package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 *YES
 */
@Config
@Autonomous
public class HoughCircles extends LinearOpMode {

    private final int rows = 640;
    private final int cols = 480;

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        //all of our movement jazz
        while (opModeIsActive()) {
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat circleMat = new Mat();
        Mat circles = new Mat();

        enum Stage
        {
            RAW,
            CIRCLES
        }

        private Stage stageToRenderToViewport = Stage.RAW;
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
            //Imgproc.cvtColor(input, circleMat, Imgproc.COLOR_RGB2GRAY);
            Core.extractChannel(rawMat, circleMat, 0);
            Imgproc.medianBlur(circleMat, circleMat, 5);
            Imgproc.HoughCircles(circleMat, circles, Imgproc.HOUGH_GRADIENT, 1.0, circleMat.rows()/16.0, 270.0, 60.0, 10, 200);
            System.out.println("Houghcircles finished");
            for (int x = 0; x < circles.cols(); x++) {
                double[] c = circles.get(0, x);
                Point center = new Point(Math.round(c[0]), Math.round(c[1]));
                Point extra = new Point(center.x + 10, center.y +10);
                // circle center
                Imgproc.circle(circleMat, center, 1, new Scalar(0,100,100), 3, 8, 0 );
                // circle outline
                int radius = (int) Math.round(c[2]);
                Imgproc.circle(circleMat, center, radius, new Scalar(255,0,255), 3, 8, 0 );
                Imgproc.putText(circleMat, "Radius: " + radius, center, Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
                double focalLength = 518.4;
                double distance = 0;
                double circleRadius = 1.875;
                distance = (circleRadius * focalLength) / radius;
                Imgproc.putText(circleMat, "Distance: " + distance, new Point(center.x + 10, center.y + 10), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
            }
            switch (stageToRenderToViewport)
            {
                case RAW:
                {
                    return rawMat;
                }
                case CIRCLES:
                {
                    return circleMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}
