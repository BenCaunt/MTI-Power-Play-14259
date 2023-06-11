package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp
//@Disabled
public class CameraSimpleDebugTeleOp extends LinearOpMode {
    private G g;
    private Pid turretPid;

    @Override
    public void runOpMode() throws InterruptedException {
        this.g = new G(hardwareMap);
        this.g.initAll();

        this.turretPid = new Pid(new Pid.Coefficients(0.002, 0.0, 0.0),
                () -> {
                    return this.g.cameraComponent.getTarget().x - C.CAMERA_WIDTH / 2.0;
                },
                factor -> {
                    this.g.turretComponent.setPower(M.clamp(factor, -0.3, 0.3));
                });

        FtcDashboard.getInstance().startCameraStream(this.g.cameraComponent.getWebcam(), 0);

        waitForStart();

        while (opModeIsActive()) {
            Point target = this.g.cameraComponent.getTarget();
            ArrayList<AprilTagDetection> detections = this.g.cameraComponent.getDetections();

            int index = 1;
            for (AprilTagDetection detection : detections) {
                telemetry.addData("Detection #" + index, detection.id);
                index++;
            }

            telemetry.addData("target", target);
            telemetry.update();

            this.g.updateAll();
            this.turretPid.update();
        }
    }
}
