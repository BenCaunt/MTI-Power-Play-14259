package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
TODO: Fix .setPosition()
 */
@Disabled
@TeleOp
public class MotorDebugTeleOp extends LinearOpMode {
    // private DcMotorExMotor motor;
    private DcMotorEx motor;

    private void initMotor() {
        // this.motor = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "turret"))
        //     .setLowerBound(0)
        //     .setUpperBound(500);
        this.motor = hardwareMap.get(DcMotorEx.class, "turret");
    }

    private void initAll() {
        this.initMotor();
    }

    private void updateMotor() {
        // this.motor.update();
    }

    private void updateAll() { this.updateMotor(); }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        while (opModeIsActive()) {
            // double power = gamepad1.left_stick_x;
            // if (Math.abs(power) > C.EPSILON) {
            //     this.motor.addPower(power);
            // } else {
            //     if (gamepad1.x) this.motor.setPosition(1.0);
            //     if (gamepad1.y) this.motor.setPosition(0.0);
            // }
            if (gamepad1.x) this.motor.setTargetPosition(500);
            if (gamepad1.y) this.motor.setTargetPosition(0);

            // telemetry.addData("Running with power?", Math.abs(power) > C.EPSILON);
            telemetry.addData("Power", this.motor.getPower());
            telemetry.addData("Target Position", this.motor.getTargetPosition());
            telemetry.addData("Current Position", this.motor.getCurrentPosition());
            telemetry.addData("Mode", this.motor.getMode());
            telemetry.update();

            this.updateAll();
        }
    }
}