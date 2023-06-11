package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.input.Controller;
import org.firstinspires.ftc.teamcode.input.controllerimpl.GamepadController;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
@Disabled
@TeleOp
public class SPMTest extends LinearOpMode {
    //
    private ServoMotor servo1;
    private Controller controller1;
    private Controller controller2;

    //Variable
    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private int clawPosition = 0;
    private int latchPosition = 0;
    private int frontArmPosition = 0;
    private int armPosition = 0;
    private double[] linSlidePositions = {0,0.5,0.7,1};
    private boolean test = false;


    private void initServo() {
        this.servo1 = new ServoMotor(hardwareMap.get(Servo.class, "servo1"))
                .setLowerBound(C.depositLB)
                .setUpperBound(C.depositUB);
    }

    private void initControllers() {
        this.controller1 = new GamepadController(gamepad1);
        this.controller2 = new GamepadController(gamepad2);

        this.controller1
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                    this.depositPosition = (this.depositPosition + 1) % C.depositPositions.length;
                    this.servo1.setPosition(C.depositPositions[depositPosition]);
                    this.updateServo();
                });
    }

    private void initPosition() {
        this.servo1.setPosition(C.depositPositions[depositPosition]);

    }
    private void initAll() {
        this.initServo();
        this.initPosition();
        this.initControllers();
    }
    private void updateServo() {
        this.servo1.update();
    }
    private void updateControllers() {
        this.controller1.update();
        this.controller2.update();
    }
    private void updateTelemetry() {
        telemetry.addData("Veer is an absolute monkey V1 XD", test);
        telemetry.update();
    }


    private void updateAll() {
//        this.updateGamepad();
        //this.updateDrivetrain();
        //this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
        this.updateControllers();
    }
    private void interact(){
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        waitForStart();

        while (opModeIsActive()) {
            this.interact();
            this.updateAll();
        }
    }
}