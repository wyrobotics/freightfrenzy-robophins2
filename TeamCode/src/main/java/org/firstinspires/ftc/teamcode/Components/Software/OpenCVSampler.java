package org.firstinspires.ftc.teamcode.Components.Software;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.BackgroundSubtractor;
import org.opencv.video.Video;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
public class OpenCVSampler {

    public OpenCvWebcam camera;

    /*
    PINK
    public static double REDMAX = 255;
    public static double REDMIN = 150;
    public static double GREENMAX = 150;
    public static double GREENMIN = 70;
    public static double BLUEMAX = 190;
    public static double BLUEMIN = 120;
     */

    public static double REDMAX = 110;
    public static double REDMIN = 60;
    public static double GREENMAX = 200;
    public static double GREENMIN = 140;
    public static double BLUEMAX = 90;
    public static double BLUEMIN = 60;

    private double contourX = 0;

    public OpenCVSampler(HardwareMap hardwareMap, Telemetry telemetry) {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

    }

    public void initOpenCVCamera() {

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
                camera.setPipeline(new OpenCVSampler.Pipeline());
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                //GPU acceleration may cause errors -J


            }
            @Override
            public void onError(int errorCode)
            {
                contourX = 69;

                camera.stopStreaming();
            }
        });

    }

    public double getPinkX(double trials) {

        double total = 0;

        for(int i = 0; i < trials; i++) {
            total += contourX;
        }

        return total / trials;

    }

    public double getPinkX() { return getPinkX(1000); }

    public class Pipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            List<MatOfPoint> pinkContours = pinkContours(input);

            double maxArea = 0;
            double newArea;

            Rect biggestRect = new Rect(0,0, 1,1);

            for(MatOfPoint contour : pinkContours) {

                newArea = Imgproc.contourArea(contour);

                if(newArea > maxArea) {

                    biggestRect = Imgproc.boundingRect(contour);

                    maxArea = newArea;

                }

                contour.release();

            }

            Imgproc.rectangle(input, biggestRect, new Scalar(200,0,0), 2);

            //Imgproc.drawContours(input, pinkContours, 2, new Scalar(0,0,255), 2, 1, hierarchy);
            //Imgproc.drawContours(input, pinkContours, -1, new Scalar(0,0,200));

            contourX = biggestRect.x;
            //contourX = input.

            pinkContours = null;

            return input;

        }

        private List<MatOfPoint> pinkContours(Mat input) {

            Mat inputRedMin = new Mat();
            Mat inputGreenMin = new Mat();
            Mat inputBlueMin = new Mat();
            Mat inputRedMax = new Mat();
            Mat inputGreenMax = new Mat();
            Mat inputBlueMax = new Mat();
            Mat pinkMask = new Mat(input.size(), 0);

            Imgproc.GaussianBlur(input, input, new Size(3,3), 0);

            List<Mat> channels = new ArrayList<Mat>();
            Core.split(input,channels);

            Imgproc.threshold(channels.get(0), inputRedMin, REDMIN, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(0), inputRedMax, REDMAX, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(channels.get(1), inputGreenMin, GREENMIN, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(1), inputGreenMax, GREENMAX, 255, Imgproc.THRESH_BINARY_INV);
            Imgproc.threshold(channels.get(2), inputBlueMin, BLUEMIN, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(channels.get(2), inputBlueMax, BLUEMAX, 255, Imgproc.THRESH_BINARY_INV);

            Core.bitwise_and(inputRedMin, inputGreenMin, pinkMask);
            Core.bitwise_and(inputBlueMin, pinkMask, pinkMask);
            Core.bitwise_and(inputRedMax, pinkMask, pinkMask);
            Core.bitwise_and(inputBlueMax, pinkMask, pinkMask);
            Core.bitwise_and(inputGreenMax, pinkMask, pinkMask);

            List<MatOfPoint> pinkContours = new ArrayList<MatOfPoint>();
            Mat hierarchy = new Mat();

            Imgproc.findContours(pinkMask, pinkContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            inputRedMin.release();
            inputGreenMin.release();
            inputBlueMin.release();
            inputRedMax.release();
            inputGreenMax.release();
            inputBlueMax.release();
            pinkMask.release();

            return pinkContours;

        }

    }



}

