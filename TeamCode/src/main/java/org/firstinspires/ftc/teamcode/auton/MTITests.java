package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

import org.firstinspires.ftc.teamcode.input.Controller;
import org.firstinspires.ftc.teamcode.input.controllerimpl.GamepadController;
import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
import org.firstinspires.ftc.teamcode.output.magnetTouch;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.DoesntResetDcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;
import org.firstinspires.ftc.teamcode.components.CameraComponent;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import org.firstinspires.ftc.robotcore.external.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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
public class MTITests extends LinearOpMode {
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

    private int state = 0;
    private AtomicBoolean test;
    private AsyncThreaded drivetrainThread;
    private AsyncThreaded linSlideThread;

    //
    private TouchSensor turretSensor;
    private Pid linSlidePid;
    private Pid pitchPid;
    private Pid turretPid;
    private Drivetrain drivetrain;
    private DoesntResetDcMotorExMotor turret;
    private DoesntResetDcMotorExMotor pitch;
    private DoesntResetDcMotorExMotor leftLinSlide;
    private DoesntResetDcMotorExMotor rightLinSlide;
    private ServoMotor claw;
    private ServoMotor frontArm;
    private ServoMotor deposit;
    private ServoMotor latch;
    private ServoMotor leftArm;
    private ServoMotor rightArm;

    private static double turretP = 3.2;
    private static double turretI = 1.2;
    private static double turretD = 0;

    private static double pitchP = 3.2;
    private static double pitchI = 1.2;
    private static double pitchD = 0;

    private static double slidesP = 3.2;
    private static double slidesI = 1.2;
    private static double slidesD = 0;

    private magnetTouch pitchTouchSensor;
    private goBildaTouchDriver turretTouchSensor;

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
    private final double firstScorePosition = 0.31;
    private final double secondScorePosition = 0.72;
    SampleMecanumDrive drive;
    Pose2d poseEstimate;

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

    private BNO055IMU imu;

    private AsyncThreaded scoring;
    private void initDrivetrain() {
        this.drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(-36,-60,Math.toRadians(-90)));
    }
    private void initMotor() {
        this.pitch = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "pitch"))
                .setLowerBound(C.pitchLB)
                .setUpperBound(C.pitchUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.turret = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "turret"))
                .setLowerBound(C.turretLB)
                .setUpperBound(C.turretUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.leftLinSlide = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "leftSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.rightLinSlide = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "rightSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    private void initPID() {
        //tune later cuh
        this.linSlidePid = new Pid(new Pid.Coefficients(slidesP, slidesI, slidesD),
                () -> this.targetLinSlidePosition - this.leftLinSlide.getCurrentPosition(),
                factor -> {
                    this.leftLinSlide.setPower(M.clamp(-factor, -1, 0.6));
                    this.rightLinSlide.setPower(M.clamp(-factor, -1, 0.6));
                });
        this.pitchPid = new Pid(new Pid.Coefficients(pitchP, pitchI, pitchD),
                () -> this.targetPitchPosition - this.pitch.getCurrentPosition(),
                factor -> {
                    this.pitch.setPower(-factor);
                });
        this.turretPid = new Pid(new Pid.Coefficients(turretP, turretI, turretD),
                () -> this.targetTurretPosition - this.turret.getCurrentPosition(),
                factor -> {
                    this.turret.setPower(factor);
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
        this.leftArm = new ServoMotor(hardwareMap.get(Servo.class, "leftLinkage"))
                .setLowerBound(C.leftArmLB)
                .setUpperBound(C.leftArmUB);
        this.rightArm = new ServoMotor(hardwareMap.get(Servo.class, "rightLinkage"))
                .setLowerBound(C.rightArmLB)
                .setUpperBound(C.rightArmUB);
        this.frontArm = new ServoMotor(hardwareMap.get(Servo.class, "frontArm"))
                .setLowerBound(C.frontArmLB)
                .setUpperBound(C.frontArmUB);
    }
    private void initSensor() {
        pitchTouchSensor = new magnetTouch(hardwareMap.get(TouchSensor.class, "touchSensor"));
        turretTouchSensor = new goBildaTouchDriver(hardwareMap.get(DigitalChannel.class, "turretTouch"));
    }

    private void initAll() {
        this.initMotor();
        this.initServo();
        this.initDrivetrain();
        this.initPID();
        this.initPosition();
        this.initSensor();

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
    private void updateDrivetrain() {
//        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
//        targetPosition = new Vector2d(poseEstimate.getX(), poseEstimate.getY());
//        Vector2d difference = targetPosition.minus(poseEstimate.vec());
//        double theta = difference.angle();
//        headingController.setTargetPosition(theta);
//        double headinginput = headingController.update(poseEstimate.getHeading());
//        if(Math.abs(gamepad1.right_stick_x) <0.05) {
//            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,headinginput));
//        }else {
//            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x));
//            drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,poseEstimate.getHeading()));
//        }
//        drive.getLocalizer().update(); //very very monke code please do not waste your time trying to understand
//        poseEstimate = drive.getLocalizer().getPoseEstimate();
//        headingController.setTargetPosition(targetAngle);

//        double headinginput = headingController.update(poseEstimate.getHeading());
//        if(Math.abs(gamepad1.right_stick_x) <0.01) { // not turning thresh
//            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,headinginput));
//        }else { // turning
//            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.8));
//            targetAngle = poseEstimate.getHeading();
//        }
        poseEstimate = drive.getLocalizer().getPoseEstimate();

        //removed auto pid holder for dp purposes
//        drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x));
        drive.getLocalizer().update();
    }

    private void updateTelemetry() {
//        updateDrivetrain();
//        telemetry.addData("this.leftLinSlide.getPower()", this.leftLinSlide.getPower());
//        telemetry.addData("linSlidePos", this.linSlidePositions[this.linSlidePosition]);
//        telemetry.addData("Veer is an absolute monkey V1 XD", this.leftLinSlide.getCurrentPosition());
//        telemetry.addData("Veer is an absolute monkey V1 XD", linSlideRTP);
//        telemetry.addData("Pitch Position", this.pitch.getCurrentPosition());
//        telemetry.addData("LinSlide target Position", this.targetLinSlidePosition);
//        telemetry.addData("ArmOut", this.armOut);
        telemetry.addData("pitch TS", this.pitchTS);
        telemetry.addData("Turret Mode", this.turretMode);
        telemetry.addData("latchEngaged" , this.latchEngaged);
//        telemetry.addData("arm Position", this.armPosition);
//        telemetry.addData("pitchRTP", this.pitchRTP);
//        telemetry.addData("pitchTargetPosition", this.targetPitchPosition);
//        telemetry.addData("limit", this.turretSensor.isPressed());
        telemetry.addData("turret pos", this.turret.getCurrentPosition());
//        telemetry.addData("rotation", poseEstimate.getHeading());
        telemetry.update();
    }
    private void updateVariable() {
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.4 -this.targetPitchPosition) ,0,1);
        targetPitchPower = gamepad2.left_stick_y;
        targetTurretPower = (gamepad1.left_trigger -gamepad1. right_trigger);
        targetLinSlidePower = 0.7*gamepad2.right_stick_y;
    }

    private void initPosition() {
        moveArm(0);
        this.targetFrontArmPosition = 0.9;
    }
    private void intakeBack(){
        this.clawOpen = false;
        this.updatePosition();
        this.updateServo();
//        sleep(250);
        long start = System.currentTimeMillis();
        while(System.currentTimeMillis()- start <= 250){
//            this.updateMotor();
        }
        frontArmPosition = 1;
        moveFrontArm();
        targetArmPosition = 0;
        this.updateServo();
    }
    private void greatReset(){
        this.linSlideRTP = true;
        this.pitchRTP = true;
        this.linSlidePosition = 0;
        this.updateAll();
        //pause
        long start = System.currentTimeMillis();
//        while(System.currentTimeMillis()- start <= 250) {
////            this.updateMotor();
//        }
        this.depositPosition = 0;
        moveDeposit();
        latchEngaged = false;
        movePitch(0.27);
        this.targetTurretPosition = 0.5;

        this.turret.update();
    }

    private void dump(){
        depositPosition = 2;
        moveDeposit();
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
    private void moveDeposit(){
        this.targetDepositPosition = (C.depositPositions[depositPosition]) - 0.0*(C.depositPositions[depositPosition] * (M.clamp(this.targetPitchPosition, 0, 0.8)));
        this.updatePosition();
        this.updateServo();
    }
    private void moveFrontArm(){
        targetFrontArmPosition = this.frontArmPositions[frontArmPosition] - (0.1*this.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
        this.updatePosition();
        this.updateServo();
    }
    private void movePitch(double position){
        this.targetPitchPosition = M.clamp(position , 0.1, 1);
    }
    private void updatePosition() {
        if(linSlideRTP) this.linSlidePid.update();
        if(pitchRTP) this.pitchPid.update();
        if(turretRTP) this.turretPid.update();
        this.frontArm.setPosition(targetFrontArmPosition);
        this.leftArm.setPosition(targetArmPosition);
        this.rightArm.setPosition(targetArmPosition);
        if(latchEngaged)this.latch.setPosition(1);
        else this.latch.setPosition(0);
        if(clawOpen)this.claw.setPosition(0.6);
        else this.claw.setPosition(0);
        this.deposit.setPosition(targetDepositPosition);
    }

    private void updateControllers() {
//        this.controller1.update();
//        this.controller2.update();
    }
    private void updateAll() { //order: DT, Var, sensor, Control, position, motor. servo, telemetry.
        this.updateVariable();
        this.updateControllers();
        this.updatePosition();
        this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
        this.updateDrivetrain();
    }
    private void interact(){
        if(Math.abs(targetTurretPower) > 0.05){
            this.turretRTP = false;
            this.turret.setPower(targetTurretPower);
        }
        if(Math.abs(targetPitchPower) > 0.05){
            this.pitchRTP = false;
            if(targetPitchPower > 0 && this.pitch.getCurrentPosition() < 0.3) targetPitchPower = 0;
            if(targetPitchPower < 0 && this.pitch.getCurrentPosition() > 1.1) targetPitchPower = 0;
            this.pitch.setPower(targetPitchPower);
        }
        if(Math.abs(targetLinSlidePower) > 0.1){
            this.linSlideRTP = false;
            this.leftLinSlide.setPower(targetLinSlidePower);
            this.rightLinSlide.setPower(targetLinSlidePower);
        }
    }
    private void preIntakeMode(){
        this.targetFrontArmPosition = 0.6;
    }
    private void scoringPosition1(){
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        this.dump();
        movePitch(0.95);
        linSlideUp();
    }
    private void scoringPosition2() {
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        this.targetDepositPosition = 0.8;
        movePitch(0.5);
        linSlideUp();
    }
    private void scoringPosition3() {
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        movePitch(0.7);
        this.targetDepositPosition = 0.8;
        linSlideUp();
        this.updateAll();
    }
    private void midPole() {
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        movePitch(0.7);
        this.targetDepositPosition = 0.95;
        this.linSlidePosition = 1;
        this.updateAll();
    }
    private void scoringPosition4() {
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        turretRTP = true;
        this.targetTurretPosition = 0.31;
        this.updateAll();
        movePitch(0.76);
        this.targetDepositPosition = 1;
        linSlideUp();
        this.updateAll();
    }
    private void scoringPosition5() {
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        turretRTP = true;
        this.targetTurretPosition = 0.58;
        this.updateAll();
        movePitch(0.46);
        this.targetDepositPosition = 0.8;
        linSlideUp();
        this.updateAll();
    }
    private void scoringPosition6() {
        pitchRTP = true;
        linSlideRTP = true;
        this.latchEngaged = true;
        turretRTP = true;
        this.targetTurretPosition = 0.86;
        this.updateAll();
        movePitch(0.55);
        this.targetDepositPosition = 1;
        linSlideUp();
        this.updateAll();
    }
    private void scoringPosition7() {
        pitchRTP = true;
        this.latchEngaged = true;
        movePitch(0);
        this.targetDepositPosition = 0.7;
        this.updateAll();
    }
    private void scoringPosition1TurretLeft(){
        pitchRTP = true;
        linSlideRTP = true;
        turretRTP = true;
        this.latchEngaged = true;
        this.dump();
        movePitch(1);
        linSlideUp();
        this.targetTurretPosition = .775;
    }
    private void scoringPosition1TurretRight(){
        pitchRTP = true;
        linSlideRTP = true;
        turretRTP = true;
        this.latchEngaged = true;
        this.dump();
        movePitch(1);
        linSlideUp();
        this.targetTurretPosition = .225;
    }
    private void linSlideUp(){
        if(linSlideHigh) this.linSlidePosition = 2;
        else this.linSlidePosition = 1;
    }

    private void linSlideReset(){
        this.pitchRTP = true;
        if(latchEngaged)this.latchEngaged = false;
        if(targetFrontArmPosition > 0.8) {
            targetFrontArmPosition = 0.7;
        }
        this.linSlidePosition = 0;
//        long start = System.currentTimeMillis();
//        while(System.currentTimeMillis()- start <= 200) {
//            this.updateDrivetrain();
//        }
        this.depositPosition = 0;
        moveDeposit();
        movePitch(0.27);
    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;//its radians you dumbass
    }
//    private void turretAutoAimTest() {
//        this.updateDrivetrain();
//        turretRTP = true;
//        turretDesiredAngle = Math.max(0,Math.min(1,(Math.toDegrees(angleWrap((Math.atan2(0 - poseEstimate.getY(), -10 - poseEstimate.getX()) - poseEstimate.getHeading() - Math.toRadians(180))))+90)/180));
//        //0 and -10 are the targets. Ill organize it later but basically we gotta make a coordinate for every pole.
//        this.targetTurretPosition = turretDesiredAngle;
//        this.updateAll();
//    }
    @Override
    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
        double a = 1;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(-90)));
        waitForStart();
        if (isStopRequested()) return;
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-36, -60, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36, -2, Math.toRadians(-90)))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(-180)))
                .build();
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if (currentDetections.size() != 0) {
//                boolean tagFound = false;
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if (tagOfInterest == null || tagOfInterest.id == LEFT) {
//                    telemetry.addLine("LEFT");
//                    randomization = 1;
//                } else if (tagOfInterest.id == MIDDLE) {
//                    telemetry.addLine("MIDDLE");
//                    randomization = 2;
//                } else {
//                    telemetry.addLine("RIGHT");
//                    randomization = 3;
//                }
//                telemetry.update();
//
//            }
//            telemetry.addLine(String.valueOf(imu.getAngularOrientation()));
//            telemetry.update();
        while(opModeInInit()) {
            this.initAll();

        }
        while(opModeIsActive()) {
            this.linSlideRTP = true;
            this.turretRTP = true;
            this.pitchRTP = true;
            if(a == 1) {
                while(a == 1) {
                    drive.followTrajectory(traj);
                    drive.followTrajectory(traj1);
                    a=2;
                }
            } else if (a == 2) {
                while(a==2) {
                    this.pitchRTP = true;
                    this.turretRTP = true;
                    targetTurretPosition = 0.44;
                    targetPitchPosition = 0.53;
                    updateAll();
                    a =3;
                }

            }else if (a==3) {
                while(a==3) {
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
                    a=4;
                }
            }

            //impliment auto aim onto far pole here or some pole to prove it works
            //x is 0, y is -24
            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
        }

}

    private boolean linSlideCheck(){
        return Math.abs(this.leftLinSlide.getCurrentPosition() - this.targetLinSlidePosition) < 0.05;
    }

    private void NLscore(){
        linSlideUp();
        sleep(100);
        depositPosition = 2;
        moveDeposit();
    }

    private void moveFrontArm1(double position){
        targetFrontArmPosition = position;
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
        this.updateMotor();
    }

    private void Ldump(){
        this.latchEngaged = false;
        this.updateAll();
    }
    private void score(){
        pitchRTP = true;
        linSlideRTP = true;
        linSlideUp();
        this.sleep(250);
        this.latchEngaged = true;
        this.updateAll();
        depositPosition = 2;
        moveDeposit();
    }

    private void preIntakeMode(int i){
        this.targetArmPosition = 0.3;
        this.targetFrontArmPosition = C.getTargetFrontArmPosition(i);
        this.updatePosition();
        this.updateServo();
    }


}