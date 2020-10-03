package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
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
public class BlueThreshold extends LinearOpMode {

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
        Mat blueGrayScale = new Mat();
        Mat blackWhite = new Mat();
        Mat output = new Mat();

        enum Stage
        {
            RAW,
            BLUEGRAY,
            BLACKWHITE,
            OUTPUT,
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
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(input, blueGrayScale, 2);
            Imgproc.cvtColor(input, blackWhite, Imgproc.COLOR_BGR2GRAY);
            Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(21, 21),new Point(10,10));
            Imgproc.morphologyEx(blueGrayScale, output, 2, element);
            Imgproc.threshold(output, output, 155, 255, Imgproc.THRESH_BINARY);
            int totalX = 0, totalY = 0, numPix = 0, leftEdge = 1000, rightEdge = 0, topEdge = 0, botEdge = 0;
            for(int row = 0; row < output.rows(); row += 10) {
                for(int col = 0; col < output.cols(); col += 10) {
                    if(output.get(row, col)[0] > 145) {
                        if(leftEdge > col) leftEdge = col;
                        if(rightEdge < col) rightEdge = col;
                        if(topEdge == 0) topEdge = row;
                        botEdge = row;
                        numPix++;
                        totalX += col;
                        totalY += row;
                    }
                }
            }
            if(numPix == 0) numPix = 1;
            if(leftEdge == 1000) leftEdge = 0;
            totalX /= numPix;
            totalY /= numPix;
            Imgproc.cvtColor(output,output, Imgproc.COLOR_GRAY2RGB);
            Imgproc.circle(output, new Point(totalX, totalY), Math.abs(((botEdge-topEdge)+(rightEdge-leftEdge))/4), new Scalar(0, 255, 0), 5);
            switch (stageToRenderToViewport)
            {
                case RAW:
                {
                    return rawMat;
                }

                case BLUEGRAY:
                {
                    return blueGrayScale;
                }

                case BLACKWHITE:
                {
                    return rawMat;
                }
                case OUTPUT:
                {
                    return output;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}
