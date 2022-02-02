package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class ComputerVision {

    private final OpenCvWebcam webcam;
    public TeamElementPipeline pipeline;

    private static final int STREAM_WIDTH = 640;
    private static final int STREAM_HEIGHT = 360;

    public ComputerVision(HardwareMap hwMap, Telemetry telemetry)
    {
        WebcamName webcamName = hwMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new TeamElementPipeline();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) { }
        });

        telemetry.addLine("Webcam Init Successful");
        telemetry.update();
    }


    public static class TeamElementPipeline extends OpenCvPipeline {
        public TeamElementPipeline() { }

        public enum ElementPosition {LEFT, MIDDLE, RIGHT}

        Point regionLeftPointA = new Point(20, 100);
        Point regionLeftPointB = new Point(80, 130);

        Point regionMidPointA = new Point(280, 100);
        Point regionMidPointB = new Point(340, 130);

        Point regionRightPointA = new Point(600, 100);
        Point regionRightPointB = new Point(540, 130);

        int avgLeft, avgMid, avgRight;

        // Volatile since accessed by OpMode thread w/o synchronization
        public volatile ElementPosition position = ElementPosition.LEFT;

        @Override
        public Mat processFrame(Mat input) {
            avgLeft = (int) Core.mean(input.submat(new Rect(regionLeftPointA, regionLeftPointB))).val[2];
            avgMid = (int) Core.mean(input.submat(new Rect(regionMidPointA, regionMidPointB))).val[2];
            avgRight = (int) Core.mean(input.submat(new Rect(regionRightPointA, regionRightPointB))).val[2];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionLeftPointA, // First point which defines the rectangle
                    regionLeftPointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionMidPointA, // First point which defines the rectangle
                    regionMidPointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    regionRightPointA, // First point which defines the rectangle
                    regionRightPointB, // Second point which defines the rectangle
                    new Scalar(0, 255, 0), // The color the rectangle is drawn in
                    1); // Thickness of the rectangle lines

//            // Record our analysis
            if (avgLeft >= avgMid && avgLeft >= avgRight) {
                position = ElementPosition.LEFT;
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        regionLeftPointA, // First point which defines the rectangle
                        regionLeftPointB, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else if (avgMid >= avgRight) {
                position = ElementPosition.MIDDLE;
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        regionMidPointA, // First point which defines the rectangle
                        regionMidPointB, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            } else {
                position = ElementPosition.RIGHT;
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        regionRightPointA, // First point which defines the rectangle
                        regionRightPointB, // Second point which defines the rectangle
                        new Scalar(0, 0, 255), // The color the rectangle is drawn in
                        -1); // Negative thickness means solid fill
            }

            return input;
        }
    }
}