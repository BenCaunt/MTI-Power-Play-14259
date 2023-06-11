package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;

@TeleOp
public class distanceSesnorTest extends LinearOpMode {
    distanceSensor LeftSensor;
    distanceSensor RightSensor;
    private double testInput1;
    private double testInput2;
    private Servo claw;
    private Servo frontArm;
    private Servo deposit;
    private Servo latch;
    private Servo leftArm;
    private Servo rightArm;

    public static final double depositLB = 0.5;
    public static final double depositUB = 1;
    public static final double latchLB = 0.38;
    public static final double latchUB = 0.65;
    public static final double frontArmLB = 0.03;
    public static final double frontArmUB = 0.9;
    public static final double clawLB = 0.45;
    public static final double clawUB = 0.85;
    public static final double leftArmLB = 0.5;
    public static final double leftArmUB = 0.75;
    public static final double rightArmLB = 0.5;
    public static final double rightArmUB = 0.75;
        @Override
    public void runOpMode() throws InterruptedException {
        LeftSensor = new distanceSensor(hardwareMap.get(AnalogInput.class, "LeftSensor"));
        RightSensor = new distanceSensor(hardwareMap.get(AnalogInput.class, "RightSensor"));
        claw = hardwareMap.get(Servo.class, "claw");
        frontArm = hardwareMap.get(Servo.class, "frontArm");
        latch = hardwareMap.get(Servo.class, "latch");
        leftArm = hardwareMap.get(Servo.class, "leftLinkage");
        rightArm = hardwareMap.get(Servo.class, "rightLinkage");
        deposit = hardwareMap.get(Servo.class, "deposit");

        claw.setPosition(clawLB);
        deposit.setPosition(depositLB);
        latch.setPosition(latchLB);
        frontArm.setPosition(frontArmLB);
        rightArm.setPosition(rightArmLB);
        leftArm.setPosition(leftArmLB);

        waitForStart();
        while(opModeIsActive()) {
            testInput1 = LeftSensor.getDistance();
            testInput2 = RightSensor.getDistance();
            claw.setPosition(clawUB);
            deposit.setPosition(depositUB);
            latch.setPosition(latchUB);
            frontArm.setPosition(frontArmUB);
            rightArm.setPosition(rightArmUB);
            leftArm.setPosition(leftArmUB);
            telemetry.addData("distance", testInput1);
            telemetry.addData("distane right", testInput2);
            telemetry.update();
        }
    }
}
