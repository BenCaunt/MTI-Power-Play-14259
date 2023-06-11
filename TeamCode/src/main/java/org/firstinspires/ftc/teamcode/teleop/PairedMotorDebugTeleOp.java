package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;
@Disabled
@TeleOp
public class PairedMotorDebugTeleOp extends LinearOpMode {
    private DcMotorExMotor motor1;
    private DcMotorExMotor motor2;
    private Pid pid;
    private double x = 0;

    private void initMotors() {
        this.motor1 = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "leftLinSlide"))
                .setLowerBound(0)
                .setUpperBound(-1500)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.motor2 = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "rightLinSlide"))
                .setLowerBound(0)
                .setUpperBound(-1500)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.pid = new Pid(new Pid.Coefficients(3.2, 1.2, 0.0),
                () -> this.x - this.motor1.getCurrentPosition(),
                factor -> {
                    this.motor1.setPower(M.clamp(-factor, -1, 0.5));
                    this.motor2.setPower(M.clamp(-factor, -1, 0.5));
                });
    }

    private void initAll() {
        this.initMotors();
    }

    private void updateMotors() {
        this.pid.update();
        this.motor1.update();
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
            if (gamepad1.x) this.x = 1;
            if (gamepad1.y) this.x = 0;

            this.updateAll();
        }
    }
}