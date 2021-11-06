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
import org.openftc.easyopencv.OpenCvInternalCamera;
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
public class BarCodeDuckPipeline extends LinearOpMode {

    private static int rows = 320;
    private static int cols = 240;
    public static int sampleWidth = 20;
    public static int sampleHeight = 10;
    public static int thresh = 210, redThresh = 137;
    public static int duckLocation = -1;
    public static int colorSpace = 41;
    public static Point leftUL = new Point(85, 120), middleUL = new Point(200, 120), rightUL = new Point(290, 120);
    public static int rotateAngle = 90;
    public static int extract = 1;

    OpenCvCamera webCam, webcam2;

    @Override
    public void runOpMode() throws InterruptedException {

        /*int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance()
                .splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.HORIZONTALLY);

        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), viewportContainerIds[0]);
        webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam2"), viewportContainerIds[1]);
        webCam.openCameraDevice();//open camera
        webcam2.openCameraDevice();//open camera
        webCam.setPipeline(new lowerCameraPipeline());//different stages
        webcam2.setPipeline(new upperCameraPipeline());//different stages
         */

        //1 camera at the moment.
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
        webCam.openCameraDevice();//open camera
        webCam.setPipeline(new duckScanPipeline());
        webCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //webcam2.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        waitForStart();
        //all of our movement jazz
        while (opModeIsActive()) {
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);
            telemetry.addData("DuckLocation: ", duckLocation);

            telemetry.update();
            sleep(100);
        }
    }

    //detection pipeline
    public static class duckScanPipeline extends OpenCvPipeline
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
            Imgproc.cvtColor(rawMat, YCRCBMat, colorSpace);
            Core.extractChannel(YCRCBMat, ExtractMat, extract);

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
            inputMat = input;
            //Mat rota = Imgproc.getRotationMatrix2D(new Point(160, 120), rotateAngle,1);
            //Imgproc.warpAffine(inputMat, inputMat, rota, new Size(320,240));
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
