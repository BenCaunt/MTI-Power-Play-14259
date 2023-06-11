package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
@Disabled
@TeleOp
public class ServoDebugTeleOp extends LinearOpMode {
    private ServoMotor motor;

    private void initMotor() {
        this.motor = new ServoMotor(hardwareMap.get(Servo.class, "servo1"));
    }

    private void initAll() {
        this.initMotor();
    }

    private void updateMotor() { this.motor.update(); }

    private void updateAll() { this.updateMotor(); }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        while (opModeIsActive()) {
            this.motor.setPosition(gamepad1.left_stick_x * 0.5 + 0.5);

            this.updateAll();
        }
    }
}