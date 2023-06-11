package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.Encoder;
import org.firstinspires.ftc.teamcode.input.encoderimpl.DcMotorExEncoder;
import org.firstinspires.ftc.teamcode.localization.Localization;
import org.firstinspires.ftc.teamcode.localization.localizationimpl.ThreeWheelLocalization;

@TeleOp
@Disabled
public class LocalizationDebugTeleOp extends LinearOpMode {
    private Drivetrain drivetrain;
    private Localization localization;
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder centerEncoder;

    private void initDrivetrain() {
        this.drivetrain = new MecanumDrivetrain(hardwareMap);
        this.drivetrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    private void initLocalization() {
        this.localization = new ThreeWheelLocalization(hardwareMap);
    }

    private void initEncoders() {
        this.leftEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        this.rightEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "FrontRight"));
        this.centerEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "BackRight"));

        this.leftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
    }

    private void initAll() {
        this.initDrivetrain();
        this.initLocalization();
        this.initEncoders();
    }

    private void updateDrivetrain() { this.drivetrain.update(); }

    private void updateLocalization() { this.localization.update(); }

    private void updateEncoders() {
        this.leftEncoder.update();
        this.rightEncoder.update();
        this.centerEncoder.update();
    }

    private void updateTelemetry() {
        telemetry.addData("this.localization.getX()", String.format("%.2f", this.localization.getX()));
        telemetry.addData("this.localization.getY()", String.format("%.2f", this.localization.getY()));
        telemetry.addData("this.localization.getR()", String.format("%.2f", this.localization.getR() / Math.PI * 180.0));
        telemetry.addData("this.leftEncoder.getX()", String.format("%.2f", this.leftEncoder.getX()));
        telemetry.addData("this.rightEncoder.getX()", String.format("%.2f", this.rightEncoder.getX()));
        telemetry.addData("this.centerEncoder.getX()", String.format("%.2f", this.centerEncoder.getX()));
        telemetry.addData("this.leftEncoder.getDx()", String.format("%.2f", this.leftEncoder.getDx()));
        telemetry.addData("this.rightEncoder.getDx()", String.format("%.2f", this.rightEncoder.getDx()));
        telemetry.addData("this.centerEncoder.getDx()", String.format("%.2f", this.centerEncoder.getDx()));
        telemetry.update();
    }

    private void updateAll() {
        this.updateDrivetrain();
        this.updateLocalization();
        this.updateEncoders();
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