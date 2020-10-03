package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
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
public class scanPipeline extends LinearOpMode {

    private final int rows = 640;
    private final int cols = 480;
    public static int sampleWidth = 3;
    public static int sampleHeight = 2;
    public static Point topCenter = new Point(320, 100);
    public static Point bottomCenter = new Point(320, 250);
    public static int thresh = 140;
    public static int stackSize = -1;
    private static double color1, color2;
    OpenCvCamera webCam;

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new StageSwitchingPipeline());//different stages
        webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        //all of our movement jazz
        while (opModeIsActive()) {
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.addData("Color1: ", color1);
            telemetry.addData("Color2: ", color2);
            telemetry.addData("StackSize: ", stackSize);

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat rawMat = new Mat();
        Mat YCRCBMat = new Mat();
        Mat ExtractMat = new Mat();
        Mat MediumRareMat = new Mat();

        enum Stage
        {
            RAW,
            YCRCB,
            EXTRACT,
            MEDIUMRARE
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
            Imgproc.cvtColor(input, YCRCBMat, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCRCBMat,ExtractMat, 2);
            Imgproc.cvtColor(ExtractMat, MediumRareMat, Imgproc.COLOR_GRAY2RGB);

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

            Imgproc.rectangle(MediumRareMat, topLeft1, bottomRight1, yellowness1 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));
            Imgproc.rectangle(MediumRareMat, topLeft2, bottomRight2, yellowness2 ? new Scalar(0, 255, 0) : new Scalar(255, 0, 0));

            switch (stageToRenderToViewport)
            {
                case RAW:
                {
                    return MediumRareMat;
                }
                case YCRCB:
                {
                    return YCRCBMat;
                }
                case EXTRACT:
                {
                    return ExtractMat;
                }
                case MEDIUMRARE:
                {
                    return MediumRareMat;
                }
                default:
                {
                    return input;
                }
            }
        }

    }
}
