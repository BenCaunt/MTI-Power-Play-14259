package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.driveimpl.PosePidDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;
import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;
@Disabled
@Autonomous
public class Lauton extends LinearOpMode {
    private Drive drive;
    private int state = 0;

    //
    private TouchSensor turretSensor;
    private Pid linSlidePid;
    private Pid pitchPid;
    private Pid turretPid;
    private Drivetrain drivetrain;
    private DcMotorExMotor turret;
    private DcMotorExMotor pitch;
    private DcMotorExMotor leftLinSlide;
    private DcMotorExMotor rightLinSlide;
    private ServoMotor claw;
    private ServoMotor frontArm;
    private ServoMotor deposit;
    private ServoMotor latch;
    private ServoMotor leftArm;
    private ServoMotor rightArm;

    //Variable
    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private int clawPosition = 0;
    private int latchPosition = 0;
    private int frontArmPosition = 1;
    private int armPosition = 2;
    private int pitchPosition;
    private double[] linSlidePositions = {0,0.5,1};
    private double pitchReset = 0;
    private double turretReset = 0;
    private double pitchLastPosition = 0;
    private double turretLastPosition = 0;

    private goBildaTouchDriver pitchTouchSensor;
    private final double[] clawPositions = { 0.0, 1.0};
    private final double[] frontArmPositions = {0,1,0.8};
    //Calculation variable!
    private double targetPitchPosition;
    private double targetTurretPosition;
    private double targetLinSlidePosition = this.linSlidePositions[linSlidePosition];
    private double targetFrontArmPosition = C.frontArmPositions[frontArmPosition];
    private double targetDepositPosition = C.depositPositions[depositPosition];
    private double targetArmPosition = 0.8;

    private boolean clawOpen = true;
    private boolean latchEngaged = true;
    private double targetPitchPower = 0;
    private double targetLinSlidePower = 0;
    private double targetTurretPower = 0;


    //Tests
    private boolean linSlideHigh = true;
    private boolean turretRTP = false;
    private boolean pitchRTP = false;
    private boolean linSlideRTP = true;
    private boolean armOut = false;
    private boolean pitchTS;
    private boolean coneIn = false;
    private boolean scorePos1 = false;
    private boolean scorePos2 = false;
    private boolean scorePos3 = false;
    private boolean turretSensorTouched;
    private boolean turretMode = false;

    private int intakeStep = 0;
    private int depositStep = 0;
    private int CycleStep = 0;
    private boolean parked = false;

    private AsyncThreaded scoring;

    private void initDrivetrain() {
        this.drivetrain = new MecanumDrivetrain(hardwareMap);
    }
    private void initMotor() {
        this.pitch = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "pitch"))
                .setLowerBound(C.pitchLB)
                .setUpperBound(C.pitchUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.turret = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "turret"))
                .setLowerBound(C.turretLB)
                .setUpperBound(C.turretUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.leftLinSlide = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "leftLinSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.rightLinSlide = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "rightLinSlide"))                .setLowerBound(C.linSlideLB)
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

    }
    private void initSensor() {
        pitchTouchSensor = new goBildaTouchDriver(hardwareMap.get(DigitalChannel.class, "pitches"));
        turretSensor = hardwareMap.get(TouchSensor.class, "Limit");

    }
    private void initPID() {
        this.targetPitchPosition = this.pitch.getCurrentPosition();
        this.targetTurretPosition = this.pitch.getCurrentPosition();
        this.linSlidePid = new Pid(new Pid.Coefficients(3.2, 1.2, 0.0),
                () -> this.targetLinSlidePosition - this.leftLinSlide.getCurrentPosition(),
                factor -> {
                    this.leftLinSlide.setPower(M.clamp(-factor, -1, 0.6));
                    this.rightLinSlide.setPower(M.clamp(-factor, -1, 0.6));
                });
        this.pitchPid = new Pid(new Pid.Coefficients(3.2, 1.2, 0.0),
                () -> this.targetPitchPosition - this.pitch.getCurrentPosition(),
                factor -> {
                    this.pitch.setPower(-factor);
                });
        this.turretPid = new Pid(new Pid.Coefficients(3.2, 1.2, 0.0),
                () -> this.targetTurretPosition - this.turret.getCurrentPosition(),
                factor -> {
                    this.turret.setPower(-factor);
                });
    }

    private void initServo() {
        this.deposit = new ServoMotor(hardwareMap.get(Servo.class, "deposit"))
                .setLowerBound(C.depositLB)
                .setUpperBound(C.depositUB);
        this.latch = new ServoMotor(hardwareMap.get(Servo.class, "latch"))
                .setLowerBound(C.latchLB)
                .setUpperBound(C.latchUB);
        this.claw = new ServoMotor(hardwareMap.get(Servo.class, "claw"))
                .setLowerBound(C.clawLB)
                .setUpperBound(C.clawUB);
        this.leftArm = new ServoMotor(hardwareMap.get(Servo.class, "leftArm"))
                .setLowerBound(C.leftArmLB)
                .setUpperBound(C.leftArmUB);
        this.rightArm = new ServoMotor(hardwareMap.get(Servo.class, "rightArm"))
                .setLowerBound(C.rightArmLB)
                .setUpperBound(C.rightArmUB);
        this.frontArm = new ServoMotor(hardwareMap.get(Servo.class, "frontArm"))
                .setLowerBound(C.frontArmLB)
                .setUpperBound(C.frontArmUB);
    }


    private void initPosition() {
        moveArm(0);
        this.targetFrontArmPosition = 0.8;

    }
    private void initAll() {
        this.initMotor();
        this.initServo();
        this.initDrivetrain();
        this.initPID();
        this.initPosition();
        this.initSensor();
        this.initDrive();
    }
    private void updateServo() {
        this.deposit.update();
        this.claw.update();
        this.latch.update();
        this.rightArm.update();
        this.leftArm.update();
        this.frontArm.update();
    }
    private void updateMotor() {
        this.leftLinSlide.update();
        this.rightLinSlide.update();
        this.pitch.update();
        this.turret.update();
    }
    private void updatePosition() {
        if(linSlideRTP) this.linSlidePid.update();
        if(pitchRTP) this.pitchPid.update();
        if(turretRTP) this.turretPid.update();
        this.frontArm.setPosition(targetFrontArmPosition);
        this.leftArm.setPosition(targetArmPosition);
        this.rightArm.setPosition(targetArmPosition);
        if(latchEngaged)this.latch.setPosition(0);
        else this.latch.setPosition(1);
        if(clawOpen)this.claw.setPosition(0);
        else this.claw.setPosition(0.6);
        this.deposit.setPosition(targetDepositPosition);
    }
    private void updateDrivetrain() {
        this.drivetrain.update();
    }
    private void updateTelemetry() {
        telemetry.addData("this.leftLinSlide.getPower()", this.leftLinSlide.getPower());
        telemetry.addData("linSlidePos", this.linSlidePositions[this.linSlidePosition]);
        telemetry.addData("Veer is an absolute monkey V1 XD", this.leftLinSlide.getCurrentPosition());
        telemetry.addData("Veer is an absolute monkey V1 XD", linSlideRTP);
        telemetry.addData("Pitch Position", this.pitch.getCurrentPosition());
        telemetry.addData("LinSlide target Position", this.targetLinSlidePosition);
        telemetry.addData("ArmOut", this.armOut);
        telemetry.addData("pitch TS", this.pitchTS);
        telemetry.addData("arm Position", this.armPosition);
        telemetry.addData("pitchRTP", this.pitchRTP);
        telemetry.addData("pitchTargetPosition", this.targetPitchPosition);
        telemetry.addData("limit", this.turretSensor.isPressed());
        telemetry.update();
    }
    private void updateVariable() {
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.4 -this.targetPitchPosition) ,0,1);
        targetPitchPower = gamepad1.right_stick_y;
        targetTurretPower = (-gamepad1.left_trigger + gamepad1.right_trigger - 0.35*gamepad2.left_trigger + 0.35*gamepad2. right_trigger);
        targetLinSlidePower = 0.7*gamepad2.right_stick_y;
    }
    private void updateSensor() {
        pitchTS = this.pitchTouchSensor.check();
        turretSensorTouched = this.turretSensor.isPressed();
    }


    private void updateAll() { //order: DT, Var, sensor, Control, position, motor. servo, telemetry.
        this.updateVariable();
        if(!parked) this.updateDrive();
        this.updateSensor();
        this.updatePosition();
        this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
    }

    private void initDrive() {
        this.drive = new PosePidDrive(hardwareMap);
    }
    private void updateDrive() {
        this.drive.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        resetDepositPosition();
        waitForStart();

        while (opModeIsActive()) {
            new Thread(() -> {
                this.updateAll();
            }).start();
                while (!this.drive.isBusy()) {
                    switch (state) {
                        case 0:
                            state = 1;
                            this.drive.addTargetY(-49);
                            break;
                        case 1:
                            state = 2;
                            this.drive.addTargetR(-Math.PI / 2.0);
                            break;
                        case 2:
                            this.targetTurretPosition = 0.4;
                            this.targetPitchPosition = 0.45;
                            this.turretRTP = true;
                            this.pitchRTP = true;
                            state = 3;
                            break;

                    }
                    updateAll();
                }
            this.updateAll();
        }
    }
    private void linSlideUp(){
        if(linSlideHigh) this.linSlidePosition = 2;
        else this.linSlidePosition = 1;
    }
    private void movePitch(){
        this.targetPitchPosition = C.pitchPositions[this.pitchPosition];
        this.updatePosition();
        this.updateServo();
    }
    private void movePitch(double position){
        this.targetPitchPosition = M.clamp(position , 0.1, 1);
    }
    private void moveTurret(double position){
        this.targetTurretPosition = position;
    }
    private void moveDeposit(){
        this.targetDepositPosition = (C.depositPositions[depositPosition]) - 0.3*(C.depositPositions[depositPosition] * (0.8 - M.clamp(this.targetPitchPosition, 0, 0.8)));
        this.updatePosition();
        this.updateServo();
    }
    private void moveFrontArm(){
        targetFrontArmPosition = this.frontArmPositions[frontArmPosition] - (0.1*this.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
        this.updatePosition();
        this.updateServo();
    }
        private void moveFrontArm1(double x){
        targetFrontArmPosition = x;
        this.updateServo();
    }
    private void moveArm(int pos){
        if(pos >= 0 && pos < 10) this.targetArmPosition = (C.armPositions[armPosition]);
        else this.targetArmPosition = 0;
        this.updatePosition();
        this.updateServo();
    }
    private void intakeOut(){
        this.clawOpen = true;
        frontArmPosition = 0;
        moveFrontArm();
        moveArm(armPosition);
    }
    private void resetPitch(){
        pitchRTP = true;
        this.targetPitchPosition = 0.5;
        this.updateMotor();
    }
    private void dump(){
        depositPosition = 2;
        moveDeposit();
    }
   private void score(){
       pitchRTP = true;
       linSlideRTP = true;
       this.latchEngaged = true;
       this.dump();
       movePitch(0.6);
       linSlideUp();
   }
    private void preIntakeMode(){
        this.targetFrontArmPosition = 0.6;
    }
    private void greatReset(){
        scorePos1 = false;
        scorePos2 = false;
        scorePos3 = false;
        sleep(100);
        this.depositPosition = 0;
        moveDeposit();
        this.linSlidePosition = 0;
        latchEngaged = false;
        movePitch(0.5);
    }
    private void linSlideReset(){
        if(targetFrontArmPosition > 0.8) targetFrontArmPosition = 0.7;
        sleep(100);
        this.depositPosition = 0;
        moveDeposit();
        this.latchEngaged = false;
        this.linSlidePosition = 0;
        movePitch(0.5);
    }
    private void intakeBack(){
        this.clawOpen = false;
        this.updatePosition();
        this.updateServo();
        sleep(250);
        frontArmPosition = 1;
        moveFrontArm();
        targetArmPosition = 0;
        this.updateServo();
    }
    private void resetDepositPosition(){
        this.targetArmPosition = 0.1;
        this.targetFrontArmPosition = 0.9;
        this.updateServo();
        updateSensor();
        if(!pitchTS) {
            while (!pitchTS) {
                updateSensor();
                telemetry.addLine("Resetting the Pitch Position");
                telemetry.addData("Pitch Touch Sensor", pitchTS);
                telemetry.update();
                this.pitch.setPower(0.25);
                this.pitch.update();
            }
        }
        this.pitch.setPower(0);
        this.pitch.update();
        this.pitch.stopAndResetEncoder();
        this.pitch.update();
        if(!turretSensorTouched) {
            while (!turretSensorTouched) {
                updateSensor();
                telemetry.addLine(" Resetting the turret Position");
                telemetry.update();
                this.turret.setPower(0.35);
                this.turret.update();
            }
        }
        this.turret.setPower(0);
        this.turret.update();
        this.turret.stopAndResetEncoder();
        this.turret.update();
        this.targetFrontArmPosition = 0.8;
        this.moveArm(-1);
        telemetry.addLine("Ready!");
        telemetry.update();
    }
}