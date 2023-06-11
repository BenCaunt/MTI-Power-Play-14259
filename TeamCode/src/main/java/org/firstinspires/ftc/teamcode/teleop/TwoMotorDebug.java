package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
@Disabled
@TeleOp
public class TwoMotorDebug extends LinearOpMode {
    private DcMotorExMotor motor1;
    private DcMotorExMotor motor2;
    private Pid pid;
    private double x = 0;

    private void initMotors() {
        this.motor1 = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "leftLinSlide"))
                .setLowerBound(0)
                .setUpperBound(500)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.motor2 = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "rightLinSlide"))
                .setLowerBound(0)
                .setUpperBound(500)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        this.pid = new Pid(new Pid.Coefficients(0.8, 0.3, 0.0),
//                () -> this.x - this.motor1.getCurrentPosition(),
//                factor -> {
//                    this.motor1.setPower(factor);
//                    this.motor2.setPower(factor);
//                });
    }

    private void initAll() {
        this.initMotors();
    }

    private void updateMotors() {
//        this.pid.update();
        this.motor1.update();
        this.motor2.setPower(this.motor1.getPower());
        this.motor2.update();
    }

    private void updateTelemetry() {
        telemetry.addData("this.motor1.getPower()", String.format("%.2f", this.motor1.getPower()));
        telemetry.addData("this.motor2.getPower()", String.format("%.2f", this.motor2.getPower()));
        telemetry.update();
    }

    private void updateAll() {
        this.updateMotors();
        this.updateTelemetry();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) this.motor2.setPosition(0);
            if (gamepad1.y) {
                this.motor1.setPosition(1);
            }

            this.updateAll();
        }
    }
}