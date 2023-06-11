package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.components.CameraComponent;
import org.firstinspires.ftc.teamcode.components.DriveComponent;
import org.firstinspires.ftc.teamcode.components.TurretComponent;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.simulation.CameraSimulation;
import org.firstinspires.ftc.teamcode.utils.M;
import org.firstinspires.ftc.teamcode.utils.Pair;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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
public class CameraBruteForceDebugTeleOp extends LinearOpMode {
    private OpenCvWebcam webcam;
    private TurretComponent turretComponent;
    private Pid turretPid;

    private static class MyPipeline extends OpenCvPipeline {
        public static Point target;

        @Override
        public Mat processFrame(Mat frame) {
            // Takes the *median* of all pixels in a 5x5 box around the each pixel,
            // and sets that pixel to this median. Great for getting rid of noise.
            Mat median = new Mat();
            Imgproc.medianBlur(frame, median, 5);

            // Convert to HSV to filter out based on yellow *hue* (the "H" in HSV)
            Mat hsv = new Mat();
            Imgproc.cvtColor(median, hsv, Imgproc.COLOR_BGR2HSV);

            // Filter out the yellow hue loosely
            Mat loose = new Mat();
            Scalar looseLowerBound = new Scalar(97, 90, 90);
            Scalar looseUpperBound = new Scalar(110, 255, 255);
            Core.inRange(hsv, looseLowerBound, looseUpperBound, loose);

            // Mask out non-yellow pixels
            Mat looseMasked = new Mat();
            Core.bitwise_and(hsv, hsv, looseMasked, loose);

            // Get the average saturation of all filtered pixels and adjust the
            // average saturation to 150
            Scalar average = Core.mean(hsv, loose);
            Mat scaled = new Mat();
            looseMasked.convertTo(scaled, -1, 150 / average.val[1], 0);

            // Filter out the saturation and value strictly
            Mat strict = new Mat();
            Scalar strictLowerBound = new Scalar(0, 140, 0);
            Scalar strictUpperBound = new Scalar(255, 255, 255);
            Core.inRange(scaled, strictLowerBound, strictUpperBound, strict);

            // Erode (shave away the outermost layer of each blob) and dilate
            // (extend the outermost layer of each blob) to get rid of outlying
            // blobs
//            Mat eroded = new Mat();
//            Imgproc.erode(filtered, eroded, kernel);
//            Mat dilated = new Mat();
//            Imgproc.dilate(eroded, dilated, kernel);

            // Generate all contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(strict, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Get bounding box of maximum height
            double maxHeight = Double.MIN_VALUE;
            Rect maxBoundingBox = null;
            for (MatOfPoint contour : contours) {
                Rect boundingBox = Imgproc.boundingRect(contour);
                if (boundingBox.height < boundingBox.width * 3) continue;
                double height = boundingBox.height;
                if (height > maxHeight) {
                    maxHeight = height;
                    maxBoundingBox = boundingBox;
                }
            }

            // Set target to largest bounding box and draw it, if it exists
            if (MyPipeline.target != null) {
                MyPipeline.target.x += (C.CAMERA_WIDTH / 2.0 - MyPipeline.target.x) * 0.1;
                MyPipeline.target.y += (C.CAMERA_HEIGHT / 2.0 - MyPipeline.target.y) * 0.1;
            }
            if (maxBoundingBox != null) {
                double x = maxBoundingBox.x + maxBoundingBox.width / 2.0;
                double y = maxBoundingBox.y + maxBoundingBox.height / 2.0;
                MyPipeline.target = new Point(x, y);
                Imgproc.rectangle(frame, maxBoundingBox, new Scalar(0, 0, 255));
            }

            return frame;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        this.webcam.setPipeline(new CameraBruteForceDebugTeleOp.MyPipeline());
        this.webcam.setMillisecondsPermissionTimeout(5000);
        this.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(C.CAMERA_WIDTH, C.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int e) {

            }
        });

        FtcDashboard.getInstance().startCameraStream(this.webcam, 0);

        this.turretComponent = new TurretComponent(null, hardwareMap);

        this.turretPid = new Pid(new Pid.Coefficients(0.005, 0.02, 0 ),
                () -> {
                    if (MyPipeline.target == null) return 0.0;
                    return MyPipeline.target.x - C.CAMERA_WIDTH / 2.0;
                },
                factor -> {
                    this.turretComponent.setPower(M.clamp(factor, -0.3, 0.3));
                });

        waitForStart();

        while (opModeIsActive()) {
            if (MyPipeline.target != null) {
                double x = MyPipeline.target.x;

                telemetry.addData("x - C.CAMERA_WIDTH / 2.0", x - C.CAMERA_WIDTH / 2.0);
                telemetry.update();
            }

            this.turretPid.update();
            this.turretComponent.update();
        }
    }
}
