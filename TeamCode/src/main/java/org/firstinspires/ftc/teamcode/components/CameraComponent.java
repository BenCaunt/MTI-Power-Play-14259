package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.simulation.CameraSimulation;
import org.firstinspires.ftc.teamcode.teleop.CameraBruteForceDebugTeleOp;
import org.firstinspires.ftc.teamcode.teleop.OpenCVDebugTeleOp;
import org.firstinspires.ftc.teamcode.utils.Pair;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;

import java.util.ArrayList;
import java.util.List;

public class CameraComponent implements Component {
    // Should apply generally, but for the time being:
    private static final Vector3 POSITION_OFFSET = new Vector3(1.75, -0.25, 7);
    private static final Vector3 ROTATION_OFFSET = new Vector3(0, -Math.PI / 12, 0);
//    private static final double[] CAMERA_MATRIX = new double[]{
//            1.34250837e+03, 0.00000000e+00, 9.96418002e+02,
//            0.00000000e+00, 1.34186945e+03, 6.09559604e+02,
//            0.00000000e+00, 0.00000000e+00, 1.00000000e+00
//    };
//    private static final double[] DIST_COEFS = new double[]{
//            -1.34858478e-02, -9.01151176e-01, -3.10528454e-04, 3.66285629e-03, 3.28190624e+00
//    };

    private Component parent;
    private OpenCvWebcam webcam;
    private MyPipeline pipeline;

    public CameraComponent(Component parent, HardwareMap hardwareMap) {
        this.parent = parent;

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        this.webcam = OpenCvCameraFactory
                .getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

//        MyPipeline.cameraMatrix.put(0, 0, CameraComponent.CAMERA_MATRIX);
//        MyPipeline.distCoefs.put(0, 0, CameraComponent.DIST_COEFS);
//
//        MyPipeline.newCameraMatrix = Calib3d.getOptimalNewCameraMatrix(MyPipeline.cameraMatrix, MyPipeline.distCoefs, new Size(C.CAMERA_WIDTH, C.CAMERA_HEIGHT), 1.0, new Size(C.CAMERA_WIDTH, C.CAMERA_HEIGHT), MyPipeline.validPixROI);

        this.pipeline = new MyPipeline();

        this.webcam.setPipeline(this.pipeline);
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
    }

    public ArrayList<AprilTagDetection> getDetections() {
        return this.pipeline.getDetections();
    }

    public Point getTarget() {
        return this.pipeline.getTarget();
    }

    private static class MyPipeline extends OpenCvPipeline {
        private static final double FX = 578.272;
        private static final double FY = 578.272;
        private static final double CX = 402.145;
        private static final double CY = 221.506;
        private static final double TAGSIZE = 0.166;
        private static final Scalar LOOSE_LOWER_BOUND = new Scalar(96, 60, 90);
        private static final Scalar LOOSE_UPPER_BOUND = new Scalar(110, 255, 255);
        private static final Scalar STRICT_LOWER_BOUND = new Scalar(0, 120, 0);
        private static final Scalar STRICT_UPPER_BOUND = new Scalar(255, 255, 255);

        private Mat cameraMatrix;
        private long nativeApriltagPtr;

        private ArrayList<AprilTagDetection> detections = new ArrayList<>();
        private Point target = new Point(C.CAMERA_WIDTH / 2.0, C.CAMERA_HEIGHT / 2.0);

        private Mat grey = new Mat();
        private Mat median = new Mat();
        private Mat hsv = new Mat();
        private Mat loose = new Mat();
        private Mat looseMasked = new Mat();
        private Mat scaled = new Mat();
        private Mat strict = new Mat();
        private List<MatOfPoint> contours = new ArrayList<>();
        private Mat hierarchy = new Mat();

        public MyPipeline() {
            constructMatrix();
            this.nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        private void constructMatrix() {
            double[] matrix = new double[]{ MyPipeline.FX, 0, MyPipeline.CX, 0, MyPipeline.FY, MyPipeline.CY, 0, 0, 1 };

            this.cameraMatrix = new Mat(3,3, CvType.CV_32FC1);
            this.cameraMatrix.put(0, 0, matrix);
        }

        public ArrayList<AprilTagDetection> getDetections() {
            return this.detections;
        }

        public Point getTarget() {
            return this.target;
        }

        @Override
        public Mat processFrame(Mat frame) {
            // Convert frame to greyscale to detect april tag better
            this.grey = new Mat();
            Imgproc.cvtColor(frame, this.grey, Imgproc.COLOR_RGBA2GRAY);

            // Get all april tag detections
            this.detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(this.nativeApriltagPtr, this.grey, MyPipeline.TAGSIZE, MyPipeline.FX, MyPipeline.FY, MyPipeline.CX, MyPipeline.CY);

            // Takes the *median* of all pixels in a 5x5 box around the each pixel,
            // and sets that pixel to this median. Great for getting rid of noise.
            this.median = new Mat();
            Imgproc.medianBlur(frame, this.median, 5);

            // Convert to HSV to filter out based on yellow *hue* (the "H" in HSV)
            this.hsv = new Mat();
            Imgproc.cvtColor(this.median, this.hsv, Imgproc.COLOR_BGR2HSV);

            // Filter out the yellow hue loosely
            this.loose = new Mat();
            Core.inRange(this.hsv, MyPipeline.LOOSE_LOWER_BOUND, MyPipeline.LOOSE_UPPER_BOUND, this.loose);

            // Mask out non-yellow pixels
            this.looseMasked = new Mat();
            Core.bitwise_and(this.hsv, this.hsv, this.looseMasked, this.loose);

            // Get the average saturation of all filtered pixels and adjust the
            // average saturation to 150
            this.scaled = new Mat();
            Scalar average = Core.mean(this.hsv, this.loose);
            this.looseMasked.convertTo(this.scaled, -1, 150 / average.val[1], 0);

            // Filter out the saturation and value strictly
            this.strict = new Mat();
            Core.inRange(this.scaled, MyPipeline.STRICT_LOWER_BOUND, MyPipeline.STRICT_UPPER_BOUND, this.strict);

            // Erode (shave away the outermost layer of each blob) and dilate
            // (extend the outermost layer of each blob) to get rid of outlying
            // blobs
//            Mat eroded = new Mat();
//            Imgproc.erode(filtered, eroded, kernel);
//            Mat dilated = new Mat();
//            Imgproc.dilate(eroded, dilated, kernel);

            // Generate all contours
            this.hierarchy = new Mat();
            this.contours.clear();
            Imgproc.findContours(this.strict, this.contours, this.hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Get bounding box of maximum height
            double minDistance = Double.MAX_VALUE;
            double maxHeight = Double.MIN_VALUE;
            Rect minBoundingBox = null;
            for (MatOfPoint contour : this.contours) {
                Rect boundingBox = Imgproc.boundingRect(contour);
                if (boundingBox.area() < 40) continue;
                double height = boundingBox.height;
                double distance = Math.abs(boundingBox.x + boundingBox.width / 2.0 - C.CAMERA_WIDTH / 2.0);
                if ((height > maxHeight) || (height == maxHeight && distance < minDistance)) {
                    maxHeight = height;
                    minDistance = distance;
                    minBoundingBox = boundingBox;
                }
            }

            // Set target to largest bounding box and draw it, if it exists
            this.target.x += (C.CAMERA_WIDTH / 2.0 - this.target.x) * 0.1;
            this.target.y += (C.CAMERA_HEIGHT / 2.0 - this.target.y) * 0.1;
            if (minBoundingBox != null) {
                double x = minBoundingBox.x + minBoundingBox.width / 2.0;
                double y = minBoundingBox.y + minBoundingBox.height / 2.0;
                this.target.x = x;
                this.target.y = y;
                Imgproc.rectangle(frame, minBoundingBox, new Scalar(0, 0, 255));
            }

            return frame;
        }
    }

//    private static class MyPipeline extends OpenCvPipeline {
//        public static Mat cameraMatrix = new Mat(3, 3, CvType.CV_64FC1);
//        public static Mat distCoefs = new Mat(1, 5, CvType.CV_64FC1);
//        public static Mat newCameraMatrix;
//        public static Rect validPixROI = new Rect();
//
//        @Override
//        public Mat processFrame(Mat frame) {
////            Mat undistorted = new Mat();
////            Calib3d.undistort(frame, undistorted, MyPipeline.cameraMatrix, MyPipeline.distCoefs, MyPipeline.newCameraMatrix);
//
////            Mat cropped = new Mat(undistorted, MyPipeline.validPixROI);
//
//            ArrayList<Pair<CameraSimulation.ScreenSpaceLine, CameraSimulation.Junction>> pairs = CameraSimulation.render();
//            for (Pair<CameraSimulation.ScreenSpaceLine, CameraSimulation.Junction> pair : pairs) {
//                CameraSimulation.ScreenSpaceLine line = pair.first;
//                CameraSimulation.Junction junction = pair.second;
//                CameraSimulation.ScreenSpacePoint a = line.a;
//                CameraSimulation.ScreenSpacePoint b = line.b;
//                Imgproc.line(frame, new Point(a.x, a.y), new Point(b.x, b.y), new Scalar(0, 0, 0));
//            }
//
//            Vector3 drivePosition = G.context.driveComponent.transformPosition(new Vector3()).multiply(40.0 / (24 * 6));
//            Vector3 cameraPosition = G.context.cameraComponent.transformPosition(new Vector3()).multiply(40.0 / (24 * 6));
//
//            Imgproc.rectangle(frame, new Point(0, 0), new Point(40, 40), new Scalar(0, 0, 0));
//            Imgproc.line(frame, new Point(40 - drivePosition.x, drivePosition.z), new Point(40 - cameraPosition.x, cameraPosition.z), new Scalar(0, 0, 0));
//            Imgproc.circle(frame, new Point(40 - drivePosition.x, drivePosition.z), (int) (8.0 * 40 / (24 * 6)), new Scalar(0, 0, 0));
//
//            return frame;
//        }
//    }

    public Component getParent() {
        return this.parent;
    }

    public OpenCvWebcam getWebcam() { return this.webcam; }

    public Vector3 transformPosition(Vector3 position) {
        return this.parent.transformPosition(position.add(CameraComponent.POSITION_OFFSET));
    }

    public Vector3 transformRotation(Vector3 rotation) {
        return this.parent.transformRotation(rotation.add(CameraComponent.ROTATION_OFFSET));
    }
}