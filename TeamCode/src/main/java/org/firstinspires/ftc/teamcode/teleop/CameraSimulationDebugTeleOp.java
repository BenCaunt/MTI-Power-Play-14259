package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.components.DriveComponent;
import org.firstinspires.ftc.teamcode.simulation.CameraSimulation;
import org.firstinspires.ftc.teamcode.utils.Pair;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

@TeleOp
@Disabled
public class CameraSimulationDebugTeleOp extends LinearOpMode {
    private G g;

    @Override
    public void runOpMode() throws InterruptedException {
        this.g = new G(hardwareMap);
        this.g.initAll();

        FtcDashboard.getInstance().startCameraStream(this.g.cameraComponent.getWebcam(), 0);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) this.g.driveComponent.setTargetX(DriveComponent.INIT_X + 10);
            if (gamepad1.y) this.g.driveComponent.setTargetX(DriveComponent.INIT_X + 0);
            if (gamepad1.a) this.g.driveComponent.setTargetY(DriveComponent.INIT_Y + 10);
            if (gamepad1.b) this.g.driveComponent.setTargetY(DriveComponent.INIT_Y + 0);
            if (gamepad1.dpad_down) this.g.driveComponent.setTargetR(Math.PI / 2);
            if (gamepad1.dpad_up) this.g.driveComponent.setTargetR(0);

            this.g.turretComponent.setPower(this.gamepad1.left_stick_y * 0.3);
            this.g.pitchComponent.setPower(this.gamepad1.right_stick_y * 1.0);
            Vector3 cameraPosition = g.cameraComponent.transformPosition(new Vector3());
            Vector3 cameraRotation = g.cameraComponent.transformRotation(new Vector3());

            telemetry.addData("cameraPosition", cameraPosition);
            telemetry.addData("cameraRotation", cameraRotation);
            telemetry.update();

            this.g.updateAll();
        }
    }
}
