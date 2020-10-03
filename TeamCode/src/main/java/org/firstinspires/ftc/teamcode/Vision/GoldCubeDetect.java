package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.CvType;
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

@Config
@Autonomous
public class GoldCubeDetect extends LinearOpMode {

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
        Mat blurredMat = new Mat();
        Mat grayMat = new Mat();
        Mat black = new Mat();
        Mat CannyMat = new Mat();
        Mat ClosedMat = new Mat();
        Mat hierarchy = new Mat();
        Mat gradX = new Mat();
        Mat gradY = new Mat();
        Mat result = new Mat();

        enum Stage
        {
            RAW,
            GRAY,
            BLUR,
            CANNY,
            OUTPUT
        }

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.RAW;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

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
            //black = new Mat(input.size(), input.type(), Scalar.all(0));
            Imgproc.cvtColor(rawMat, grayMat, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(grayMat,grayMat, 0);
            Imgproc.GaussianBlur(grayMat, blurredMat, new Size(5,5),Imgproc.RETR_TREE,Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.threshold(blurredMat, blurredMat, 169, 255, Imgproc.THRESH_BINARY);
            Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(13,13), new Point(10,10));
            Imgproc.morphologyEx(blurredMat, blurredMat,3, element);
            Imgproc.Canny(blurredMat, CannyMat, 30,90, 3, false);
            //input.copyTo(black,CannyMat);

            /*rawMat = input;
            Imgproc.GaussianBlur(rawMat, rawMat, new Size(3,3),0,0);
            Imgproc.cvtColor(rawMat,grayMat,Imgproc.COLOR_BGR2GRAY);
            Imgproc.Sobel(grayMat,gradX, CvType.CV_16S,1,0,3,1,0, Core.BORDER_DEFAULT);
            Imgproc.Sobel(grayMat,gradY, CvType.CV_16S,0,1,3,1,0, Core.BORDER_DEFAULT);
            Core.convertScaleAbs(gradX,gradX);
            Core.convertScaleAbs(gradY,gradY);
            Core.addWeighted(gradX,0.5,gradY,0.5,0, result);
             */

            /*rawMat = input;
            Imgproc.cvtColor(rawMat,rawMat,Imgproc.COLOR_BGR2YCrCb);
            Imgproc.GaussianBlur(rawMat, grayMat, new Size(3,3),0,0);
            Core.extractChannel(grayMat,grayMat, 0);
            Imgproc.Sobel(grayMat,gradX, CvType.CV_16S,1,0,3,1,0, Core.BORDER_DEFAULT);
            Imgproc.Sobel(grayMat,gradY, CvType.CV_16S,0,1,3,1,0, Core.BORDER_DEFAULT);
            Core.convertScaleAbs(gradX,gradX);
            Core.convertScaleAbs(gradY,gradY);
            Core.addWeighted(gradX,0.5,gradY,0.5,0, result);
             */

            //System.out.println("2");
            //Imgproc.cvtColor(ClosedMat,ClosedMat, Imgproc.COLOR_RGB2GRAY);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(CannyMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Mat drawing = Mat.zeros(CannyMat.size(), CvType.CV_8UC3);
            //System.out.println(contours.size());

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
            if(contours.size() != 0) System.out.println(contoursPoly[0].toArray()[0] + " and " + contoursPoly[0].toArray()[contoursPoly[0].rows()/2]);
            for (int i = 1; i < contours.size(); i++) {
                double numberPoints = contoursPoly[i].rows() -1;
                System.out.println(contours.size());
                Scalar color = new Scalar(0, 255, 0);
                Imgproc.drawContours(drawing, contoursPolyList, i, color);
                //Imgproc.rectangle(drawing, boundRect[i].tl(), boundRect[i].br(), color, 2);
                Imgproc.circle(drawing, centers[i], (int) radius[i][0], color, 2);
                for(int n = 1; n < contoursPoly[i].rows()-1; n++) {
                    Point SlopePoint = contoursPoly[i].toArray()[n];
                    //Imgproc.putText(drawing, "" + n, new Point(SlopePoint.x, SlopePoint.y), Imgproc.FONT_HERSHEY_DUPLEX, 0.5, new Scalar(255,0,0));
                    double Slope1 = (SlopePoint.y - contoursPoly[i].toArray()[n-1].y)/(SlopePoint.x - contoursPoly[i].toArray()[n-1].x);
                    double Slope2 = (SlopePoint.y - contoursPoly[i].toArray()[n+1].y)/(SlopePoint.x - contoursPoly[i].toArray()[n+1].x);
                    if(Math.abs(Slope1 - Slope2) < 10){
                        numberPoints--;
                    }
                }
                if(numberPoints >= 3 && numberPoints <= 5) {
                    Imgproc.putText(drawing, "Square: Points= " + numberPoints, boundRect[i].tl(), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
                }

                //Imgproc.putText(drawing, "C0", new Point(contoursPoly[i].toArray()[0].x, contoursPoly[i].toArray()[0].y), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
                //Imgproc.putText(drawing, "C1", new Point(contoursPoly[i].toArray()[contoursPoly[i].rows()/2].x, contoursPoly[i].toArray()[contoursPoly[i].rows()/2].y), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
                //if(i < contours.size() - 1) Imgproc.putText(drawing, "C1", new Point(contours.get(i + 1).toArray()[0].x, contours.get(i + 1).toArray()[0].y), Imgproc.FONT_HERSHEY_DUPLEX, 0.7, new Scalar(255,0,0));
            }

            /*for (int i = 0; i < contours.size(); i++) {
                Scalar color = new Scalar(0, 255, 255);
                Imgproc.drawContours(drawing, contours, i, color, 2, Imgproc.LINE_8, hierarchy, 0, new Point());
            }
             */

            switch (stageToRenderToViewport) {
                case RAW: {
                    return rawMat;
                }
                case BLUR: {
                    return blurredMat;
                }
                case GRAY: {
                    return grayMat;
                }
                case CANNY: {
                    return CannyMat;
                }
                //case CLOSED: {
                //    return ClosedMat;
                //}
                case OUTPUT: {
                    return drawing;
                }
                default:
                {
                    return input;
                }
            }
        }

    }



}
