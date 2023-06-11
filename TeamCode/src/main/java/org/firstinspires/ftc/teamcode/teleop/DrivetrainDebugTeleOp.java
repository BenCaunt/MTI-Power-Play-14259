package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
@Disabled
@TeleOp
public class DrivetrainDebugTeleOp extends LinearOpMode {
    private Drivetrain drivetrain;

    private void initDrivetrain() {
        this.drivetrain = new MecanumDrivetrain(hardwareMap);
    }

    private void initAll() {
        this.initDrivetrain();
    }

    private void updateDrivetrain() { this.drivetrain.update(); }

    private void updateTelemetry() {
        telemetry.addData("this.drivetrain.getPowerX()", this.drivetrain.getPowerX());
        telemetry.addData("this.drivetrain.getPowerY()", this.drivetrain.getPowerY());
        telemetry.addData("this.drivetrain.getPowerR()", this.drivetrain.getPowerR());
        telemetry.update();
    }

    private void updateAll() {
        this.updateDrivetrain();
        this.updateTelemetry();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        while (opModeIsActive()) {
            this.drivetrain.addPowerX(gamepad1.left_stick_x);
            this.drivetrain.addPowerY(gamepad1.left_stick_y);
            this.drivetrain.addPowerR(gamepad1.right_stick_x);

            this.updateAll();
        }
    }
}