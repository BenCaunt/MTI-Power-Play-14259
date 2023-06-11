package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

@Autonomous
@Disabled
public class RightSideCircuitAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int randomization = 1;
    double fx = 578.272;
    double fy = 578.272;
    double tagsize = 0.166;
    double cx = 402.145;
    double cy = 221.506;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    private Drive drive;
    private int state = 0;
    private AtomicBoolean test;
    private AsyncThreaded drivetrainThread;
    private AsyncThreaded linSlideThread;
    private AsyncThreaded motorThread;

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
    private double linSlidePower;

    private int intakeStep = 0;
    private int depositStep = 0;
    private int CycleStep = 0;

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
                    this.linSlidePower = -factor;
                    this.leftLinSlide.setPower(M.clamp(-factor, -1, 0.5));
                    this.rightLinSlide.setPower(M.clamp(-factor, -1, 0.5));
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
        moveArm(-1);
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
        this.pitchPid.update();
        this.turretPid.update();
        this.pitch.update();
        this.turret.update();
    }
    private void updateLinSlide(){
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.32  -this.targetPitchPosition) ,0,1);
        this.linSlidePid.update();
        this.leftLinSlide.update();
        this.rightLinSlide.update();
    }
    private void updatePosition() {
        this.frontArm.setPosition(targetFrontArmPosition);
        this.leftArm.setPosition(targetArmPosition);
        this.rightArm.setPosition(targetArmPosition);
        if(latchEngaged)this.latch.setPosition(1);
        else this.latch.setPosition(0);
        if(clawOpen)this.claw.setPosition(0.6);
        else this.claw.setPosition(0);
        this.deposit.setPosition(targetDepositPosition);
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
        targetPitchPower = gamepad1.right_stick_y;
        targetTurretPower = (-gamepad1.left_trigger + gamepad1.right_trigger - 0.35*gamepad2.left_trigger + 0.35*gamepad2. right_trigger);
        targetLinSlidePower = 0.7*gamepad2.right_stick_y;
    }
    private void updateSensor() {
        pitchTS = this.pitchTouchSensor.check();
        turretSensorTouched = this.turretSensor.isPressed();
    }
    private void initAsync() {
        drivetrainThread = new AsyncThreaded(() -> {})
                .then(() -> {
                    this.drive.addTargetX(-50);
                    while (this.drive.isBusy() && !AsyncThreaded.stopped) this.drive.update();
                    C.parked.set(true);
                    while (C.parked.get() && !AsyncThreaded.stopped) this.drive.update();
                });
        linSlideThread = new AsyncThreaded(() -> {})
                .then(() -> {
                    while (!AsyncThreaded.stopped) {
                        this.updateLinSlide();
                    }
                });
        motorThread = new AsyncThreaded(() -> {})
                .then(() -> {
                    while (!AsyncThreaded.stopped) {
                        this.updateMotor();
                    }
                });
    }


    private void updateAll() { //order: DT, Var, sensor, Control, position, motor. servo, telemetry.
        this.updateVariable();
        this.updateSensor();
        this.updatePosition();
        this.updateServo();
        this.updateTelemetry();
    }

    private void initDrive() {
        this.drive = new PosePidDrive(hardwareMap);
    }
    private void updateDrive() {
        this.drive.update();
    }
    private AtomicInteger Randomization = new AtomicInteger(1);

    @Override
    public void runOpMode() throws InterruptedException {
        this.latchEngaged = false;
        this.initAll();
        resetDepositPosition();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                    telemetry.addLine("LEFT");
                    randomization = 1;
                } else if (tagOfInterest.id == MIDDLE) {
                    telemetry.addLine("MIDDLE");
                    randomization = 2;
                } else {
                    telemetry.addLine("RIGHT");
                    randomization = 3;
                }
                telemetry.update();

            }
        }
        if (opModeIsActive()) {
            AsyncThreaded.stopped = false;
            this.initAsync();
            this.Randomization.set(randomization);
            this.drivetrainThread.run();
            this.linSlideThread.run();
            this.targetTurretPosition = 0.3;
            this.targetPitchPosition = 0.6;
            while(!C.parked.get()) this.updateAll();
            score();
            updateAll();
            preIntakeMode();
            while(!linSlideCheck()){
                updateAll();
            }
            sleep(100);
            dump();
            this.updateAll();
            sleep(250);
            this.armPosition = 5;
            linSlideReset();
            intakeOut(C.getTargetFrontArmPosition(5));
            this.updateAll();
            sleep(150);
            targetTurretPosition = 0.63;
            targetPitchPosition = 0.50;
            for(int j = 4; j >= 0; j--){
                sleep(75);
                intakeBack();
                updateAll();
                sleep(300);
                this.clawOpen = true;
                this.updateAll();
                sleep(150);
                if(j == 0) break;
                preIntakeMode();
                this.updateAll();
                this.score();
                updateAll();
                while(!linSlideCheck()){
                    this.updateAll();
                }
                dump();
                this.updateAll();
                sleep(250);
                linSlideReset();
                intakeOut(C.getTargetFrontArmPosition(j));
                this.updateAll();
                sleep(100);
            }
            this.updateAll();
            this.score();
            updateAll();
            while(!linSlideCheck()){
                this.updateAll();
            }
            dump();
            this.updateAll();
            sleep(250);
            linSlideReset();
            this.updateAll();
            sleep(100);
            intakeBack();
            this.updateAll();
            this.targetTurretPosition = 0.1;
            this.targetPitchPosition = 0.2;
            sleep(200);
            this.turretRTP = true;
            this.pitchRTP = true;
            C.parked.set(false);
            AsyncThreaded.stopped = true;
            if(randomization == 1){
                this.drive.addTargetY(-24);
            }
            else if(randomization == 3){
                this.drive.addTargetY(24);
            }
            else this.drive.addTargetY(0);
            while(this.drive.isBusy()){
                this.updateAll();
                this.updateDrive();
            }
            this.drive.addTargetR(Math.toRadians(-90));
            while(this.drive.isBusy()){
                this.updateAll();
                this.updateDrive();
            }
            for(int i = 0; i< 5 ;i++){
                this.updateAll();
                this.updateDrive();
            }
        }
    }
    private boolean linSlideCheck(){
        return Math.abs(this.leftLinSlide.getCurrentPosition() - this.targetLinSlidePosition) < 0.05;
    }
    private boolean pitchCheck(){
        return Math.abs(this.pitch.getCurrentPosition() - this.targetPitchPosition) < 0.05;
    }
    private boolean turretCheck(){
        return Math.abs(this.turret.getCurrentPosition() - this.targetTurretPosition) < 0.05;
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
        this.targetDepositPosition = (C.depositPositions[depositPosition]) - 0.2;
        this.updatePosition();
        this.updateServo();
    }
    private void moveFrontArm1(double position){
        targetFrontArmPosition = position;
        this.updateServo();
    }
    private void moveArm(int pos){
        if(pos >= 0 && pos < 10) this.targetArmPosition = (C.armPositions[armPosition]);
        else this.targetArmPosition = 0.05;
        this.updatePosition();
        this.updateServo();
    }
    private void intakeOut(double position){
        if(position < 0) position = 0;
        this.clawOpen = true;
//        frontArmPosition = 0;
        moveFrontArm1(position);
        moveArm(armPosition);
    }
    private void resetPitch(){
        pitchRTP = true;
        this.targetPitchPosition = 0.5;
    }
    private void preDump(){
        depositPosition = 1;
        moveDeposit();
    }
    private void dump(){
        this.latchEngaged = false;
        this.updateAll();
    }
    private void score(){
        pitchRTP = true;
        linSlideRTP = true;
        linSlideUp();
        this.sleep(150);
        this.latchEngaged = true;
        this.updateAll();
        depositPosition = 2;
        moveDeposit();
    }
    private void preIntakeMode(){
        this.targetFrontArmPosition = 0.2;
        this.targetArmPosition = 0.3;
        this.updatePosition();
        this.updateServo();
    }
    private void greatReset(){
        this.depositPosition = 0;
        moveDeposit();
        this.linSlidePosition = 0;
        latchEngaged = false;
        movePitch(0.5);
    }
    private void linSlideReset(){
        this.depositPosition = 0;
        moveDeposit();
        this.latchEngaged = false;
        this.linSlidePosition = 0;
        movePitch(0.5);
    }
    //    private double getTargetFrontArmPosition(int i){
//        if(i == 4) return 0.09;
//        else if(i == 3)return 0.07;
//        else if(i == 2)return 0.04;
//        else if(i == 1)return 0.00;
//        else if(i == 0) return 0.0;
//        else if(i == 5) return 0.15;
//        else return 0;
//    }
    private void intakeBack(){
        this.clawOpen = false;
        this.updatePosition();
        this.updateServo();
        sleep(150);
        moveFrontArm1(0.9);
        updateAll();
        sleep(150);
        moveArm(-1);
        this.updateAll();
    }
    private void resetDepositPosition(){
        this.targetFrontArmPosition = 0.7;
        this.updatePosition();
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