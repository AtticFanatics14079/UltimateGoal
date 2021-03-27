package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
public class twoScanPipeline extends LinearOpMode {

    private final int rows = 640;
    private final int cols = 480;
    public static int sampleWidth = 30;
    public static int sampleHeight = 3;
    public static Point topCenter = new Point(510, 420);
    public static Point bottomCenter = new Point(510, 350);
    public static Point leftBar1 = new Point(442, 360), leftBar2 = new Point(451, 436), rightBar1 = new Point(198, 359), rightBar2 = new Point(207, 435);
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

    @Override
    public void runOpMode() throws InterruptedException {

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
        webcam2.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        initDetect = false;
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
    static class lowerCameraPipeline extends OpenCvPipeline
    {
        private double middle = -1, offset = 0;

        Mat rawMat = new Mat();
        Mat YCRCBMat = new Mat();
        Mat ExtractMat = new Mat();
        Mat MediumRareMat = new Mat();
        Mat redMat = new Mat();

        enum Stage
        {
            RAW,
            YCRCB,
            EXTRACT,
            MEDIUMRARE,
            RED
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
            else if(!lameMode){
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
            else MediumRareMat = input;
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

        private Stage stageToRenderToViewport = Stage.INTER;
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
            if(lameMode) return input;
            inputMat = input;
            Mat rota = Imgproc.getRotationMatrix2D(new Point(160, 120), rotateAngle,1);
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
            Point[] centers = new Point[contours.size()];
            float[][] radius = new float[contours.size()][1];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
                Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
            }
            List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
            for (MatOfPoint2f poly : contoursPoly) {
                contoursPolyList.add(new MatOfPoint(poly.toArray()));
            }
            for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(outputMat, contoursPolyList, i, new Scalar(0,255,0), 3);
                Imgproc.rectangle(outputMat, boundRect[i].tl(), boundRect[i].br(), new Scalar(255,0,0), 2);
                Imgproc.line(outputMat, new Point((boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x)/2, 0), new Point((boundRect[contours.size()-1].tl().x+boundRect[contours.size()-1].br().x)/2, 480), new Scalar(0,0,255), 3);
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
