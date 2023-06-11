package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Disabled
public class OpenCVDebugTeleOp extends LinearOpMode {
    private OpenCvWebcam webcam;

    private static class DebugPipeline extends OpenCvPipeline {
        private static final int KERNEL_RADIUS = 3;
        private static Mat kernel;

        public static void initKernel() {
            kernel = new Mat(KERNEL_RADIUS * 2 + 1, KERNEL_RADIUS * 2 + 1, CvType.CV_8U);
            for (int i = -KERNEL_RADIUS; i <= KERNEL_RADIUS; i++) {
                for (int j = -KERNEL_RADIUS; j <= KERNEL_RADIUS; j++) {
                    int[] data = new int[1];
                    if (i * i + j * j <= KERNEL_RADIUS * KERNEL_RADIUS) data[0] = 1;
                    kernel.put(i + KERNEL_RADIUS, j + KERNEL_RADIUS, data);
                }
            }
        }

        @Override
        public Mat processFrame(Mat frame) {
            // Convert to HSV
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

            // Filter out yellow
            Mat filtered = new Mat();
            Scalar lowerBound = new Scalar(80, 160, 140);
            Scalar upperBound = new Scalar(115, 255, 255);
            Core.inRange(hsv, lowerBound, upperBound, filtered);

            // Erode and dilate
            Mat eroded = new Mat();
            Imgproc.erode(filtered, eroded, kernel);
            Mat dilate = new Mat();
            Imgproc.dilate(eroded, dilate, kernel);

            // Generate contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(dilate, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw bounding boxes
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(frame, rect, new Scalar(0, 0, 255), 4);
            }

            return frame;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        this.webcam.setPipeline(new OpenCVDebugTeleOp.DebugPipeline());

        this.webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int e) {

            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("this.webcam.getFrameCount()", this.webcam.getFrameCount());
            telemetry.update();
        }
    }
}
