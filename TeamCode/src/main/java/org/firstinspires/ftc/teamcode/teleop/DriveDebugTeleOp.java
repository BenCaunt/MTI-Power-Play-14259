package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.driveimpl.PosePidDrive;

@TeleOp
@Disabled
public class DriveDebugTeleOp extends LinearOpMode {
    private Drive drive;
    private double x = 0;
    private double y = 0;
    private double r = 0;
    private double distance;
    private double rotation;
    private double distanceThreshold = C.DISTANCE_THRESHOLD;
    private double rotationThreshold = C.ROTATION_THRESHOLD;

    // From PosePidDrive.isBusy() implementation
    private boolean isBusy() {
        double errorX = this.drive.getErrorX();
        double errorY = this.drive.getErrorY();
        double errorR = this.drive.getErrorR();
        distance = Math.sqrt(errorX * errorX + errorY * errorY);
        rotation = Math.abs(errorR);
        return distance > this.distanceThreshold || rotation > this.rotationThreshold;
    }

    private void initDrive() {
        this.drive = new PosePidDrive(hardwareMap);
    }

    private void initAll() {
        this.initDrive();
    }

    private void updateDrive() {
        this.drive.update();
    }

    private void updateTelemetry() {
        telemetry.addData("this.drive.getErrorX()", this.drive.getErrorX());
        telemetry.addData("this.drive.getErrorY()", this.drive.getErrorY());
        telemetry.addData("this.drive.getErrorR()", this.drive.getErrorR());
        telemetry.addData("this.drive.isBusy()", this.drive.isBusy());
        telemetry.addData("this.distanceThreshold", this.distanceThreshold);
        telemetry.addData("this.rotationThreshold", this.rotationThreshold);
        telemetry.addData("this.distance", this.distance);
        telemetry.addData("this.rotation", this.rotation);
        telemetry.addData("this.isBusy()", this.isBusy());
        telemetry.update();
    }

    private void updateAll() {
        this.updateDrive();
        this.updateTelemetry();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) this.x = 10;
            if (gamepad1.y) this.x = 0;
            if (gamepad1.a) this.y = 10;
            if (gamepad1.b) this.y = 0;
            if (gamepad1.dpad_down) this.r = Math.PI / 2.0;
            if (gamepad1.dpad_up) this.r = 0;
            this.distanceThreshold = Math.max(this.distanceThreshold + gamepad2.left_stick_y, 0.0);
            this.rotationThreshold = Math.max(this.rotationThreshold + gamepad2.right_stick_y, 0.0);

            this.drive.setTargetX(this.x);
            this.drive.setTargetY(this.y);
            this.drive.setTargetR(this.r);
            this.updateAll();
        }
    }
}