package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;
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
public class Morphology extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

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
        runtime.reset();
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
        Mat openedMat = new Mat();
        Mat closedMat = new Mat();
        Mat lineMat = new Mat();


        enum Stage
        {//color difference. greyscale
            OPENED,
            CLOSED,
            LINES,
            RAW_IMAGE//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
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

            Mat gray = new Mat();

            Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(21,21),new Point(10,10));
            Imgproc.morphologyEx(rawMat,openedMat,2, element);
            Imgproc.morphologyEx(rawMat,closedMat,4, element);
            Imgproc.cvtColor(rawMat,gray, Imgproc.COLOR_BGR2GRAY);
            Core.bitwise_not(gray,gray);
            Imgproc.adaptiveThreshold(gray,lineMat, 255, Imgproc.ADAPTIVE_THRESH_MEAN_C, Imgproc.THRESH_BINARY, 15, -2);

            Mat horizontalStructure = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(lineMat.cols()/50.0,5));
            Imgproc.erode(lineMat,lineMat,horizontalStructure);
            Imgproc.dilate(lineMat,lineMat,horizontalStructure);


            switch (stageToRenderToViewport)
            {
                case OPENED:
                {
                    return openedMat;
                }

                case CLOSED:
                {
                    return closedMat;
                }

                case LINES:
                {
                    return lineMat;
                }

                case RAW_IMAGE:
                {
                    return rawMat;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}
