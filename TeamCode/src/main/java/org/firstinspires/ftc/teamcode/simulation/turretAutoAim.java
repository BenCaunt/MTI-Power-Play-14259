package org.firstinspires.ftc.teamcode.simulation;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.Async;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;
import org.firstinspires.ftc.teamcode.input.controllerimpl.GamepadController;
import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
import org.firstinspires.ftc.teamcode.output.magnetTouch;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;


@TeleOp
public class turretAutoAim extends LinearOpMode {
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
//    private PIDFController translationalController = new PIDFController(SampleMecanumDrive.TRANSLATIONAL_PID);

    //    public static double DISTANCE = 20;
    private Vector2d targetPosition = new Vector2d(0, 0);

    private AsyncThreaded drivetrainThread;
    private Pid pitchPid;
    private Pid turretPid;
    private Pid linSlidePid;
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

    Pose2d poseEstimate;
    //Variable
    private double targetAngle = 0;

    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private int clawPosition = 0;
    private int latchPosition = 0;
    private int frontArmPosition = 1;
    private int armPosition = 2;
    private int pitchPosition;
    private double[] linSlidePositions = {0,0.45,1};
    private double pitchReset = 0;
    private double turretReset = 0;
    private double pitchLastPosition = 0;
    private double turretLastPosition = 0;

    private magnetTouch pitchTouchSensor;
    private final double[] clawPositions = { 0.0, 1.0};
    private final double[] frontArmPositionas = {0,1,0.8};
    //Calculation variable!
    private double targetPitchPosition;
    private double targetTurretPosition;
    private double targetLinSlidePosition = this.linSlidePositions[linSlidePosition];
    private double targetFrontArmPosition = C.frontArmPositions[frontArmPosition];
    private double targetDepositPosition = C.depositPositions[depositPosition];
    private double targetArmPosition = 0.8;
    private double lastTurretPosition = 0.5;
    private final double[] frontArmPositions = {0,1,0.8};

    private boolean clawOpen = true;
    private boolean latchEngaged = false;
    private double targetPitchPower = 0;
    private double targetLinSlidePower = 0;
    private double targetTurretPower = 0;
    private double turretDesiredAngle;

    //Tests
    private boolean linSlideHigh = true;
    private boolean turretRTP = false;
    private boolean pitchRTP = false;
    private boolean linSlideRTP = true;
    private boolean armOut = false;
    private boolean pitchTS;
    private boolean turretSensorTouched;
    private boolean turretMode = false;
    private int scorePos = 1;
    private int autoLeftSequence = 1;
    private int intakeStep = 1;
    private int depositStep = 1;
    private int cycleStep = 1;
    SampleMecanumDrive drive;

    private void initDrivetrain() {
        this.drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        headingController.setInputBounds(-Math.PI, Math.PI);
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
        this.leftLinSlide = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "leftSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.rightLinSlide = new DcMotorExMotor(hardwareMap.get(DcMotorEx.class, "rightSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    private void initPID() {
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
                    this.turret.setPower(factor);
                });
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,Math.toRadians(0)));
        headingController.setInputBounds(-Math.PI, Math.PI);
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
    }

    private void initAll() {
        this.initMotor();
        this.initServo();
        this.initDrivetrain();
        this.initPID();
        this.initPosition();
        this.initSensor();
        this.initAsync();
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
        poseEstimate = drive.getLocalizer().getPoseEstimate();
        headingController.setTargetPosition(targetAngle);

        double headinginput = headingController.update(poseEstimate.getHeading());
        if(Math.abs(gamepad1.right_stick_x) <0.05) { // not turning
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,headinginput));
        }else { // turning
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x, -gamepad1.right_stick_x));
            targetAngle = poseEstimate.getHeading();
        }
        drive.getLocalizer().update();
    }
    private void updateTelemetry() {
//        telemetry.addData("this.leftLinSlide.getPower()", this.leftLinSlide.getPower());
//        telemetry.addData("linSlidePos", this.linSlidePositions[this.linSlidePosition]);
//        telemetry.addData("Veer is an absolute monkey V1 XD", this.leftLinSlide.getCurrentPosition());
//        telemetry.addData("Veer is an absolute monkey V1 XD", linSlideRTP);
//        telemetry.addData("Pitch Position", this.pitch.getCurrentPosition());
//        telemetry.addData("LinSlide target Position", this.targetLinSlidePosition);
//        telemetry.addData("ArmOut", this.armOut);
//        turretAutoAimTest();
        telemetry.addData("pitch TS", this.pitchTS);
        telemetry.addData("Turret Mode", this.turretMode);
        telemetry.addData("latchEngaged" , this.latchEngaged);
//        telemetry.addData("arm Position", this.armPosition);
//        telemetry.addData("pitchRTP", this.pitchRTP);
//        telemetry.addData("pitchTargetPosition", this.targetPitchPosition);
//        telemetry.addData("limit", this.turretSensor.isPressed());
        telemetry.addData("robot x", poseEstimate.getX());
        telemetry.addData("robot y", poseEstimate.getY());
        telemetry.addData("robot r", poseEstimate.getHeading());
        telemetry.addData("turret pos", this.turret.getCurrentPosition());
        telemetry.addData("turret target pos", turretDesiredAngle);
        telemetry.update();
    }
    private void updateVariable() {
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.4 -this.targetPitchPosition) ,0,1);
        targetPitchPower = gamepad1.right_stick_y;
        //targetTurretPower = (-gamepad1.left_trigger + gamepad1.right_trigger - 0.35*gamepad2.left_trigger + 0.35*gamepad2. right_trigger);
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
        while(System.currentTimeMillis()- start <= 250) {
            this.updateDrivetrain();
        }
        frontArmPosition = 1;
        moveFrontArm();
        targetArmPosition = 0;
        this.updateServo();
    }
    private void greatReset(){
        this.linSlideRTP = true;
        this.depositPosition = 0;
        moveDeposit();
        this.linSlidePosition = 0;
        latchEngaged = false;
        movePitch(0.5);
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
    private void updateSensor() {
        pitchTS = this.pitchTouchSensor.check();
    }

    private void updateAll() { //order: DT, Var, sensor, Control, position, motor. servo, telemetry.
        this.updateVariable();
        this.updateSensor();
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
        movePitch(0.6);
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
        linSlideRTP = true;
        this.latchEngaged = true;
        movePitch(0.2);
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
        this.targetTurretPosition = 0.86;
    }
    private void scoringPosition1TurretRight(){
        pitchRTP = true;
        linSlideRTP = true;
        turretRTP = true;
        this.latchEngaged = true;
        this.dump();
        movePitch(1);
        linSlideUp();
        this.targetTurretPosition = .14;
    }
    private void linSlideUp(){
        if(linSlideHigh) this.linSlidePosition = 2;
        else this.linSlidePosition = 1;
    }

    private void linSlideReset(){
        if(targetFrontArmPosition > 0.8) {
            targetFrontArmPosition = 0.7;
            long start = System.currentTimeMillis();
            while(System.currentTimeMillis()- start <= 100) {
                this.updateDrivetrain();
            }
        }
        this.depositPosition = 0;
        moveDeposit();
        if(latchEngaged)this.latchEngaged = false;
        this.linSlidePosition = 0;
        movePitch(0.5);
    }

    private void resetDepositPosition(){
        this.targetArmPosition = 0.1;
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
        if(!gamepad1.a) {
            while (!gamepad1.a) {
                turret.setPower(gamepad1.right_stick_x);
                telemetry.addData("use the right joystick to rotate the turret ~45 degrees to the left assuming the front arm does not face you (ignore this im too lazy to add the auto tuner in)", turret.getPower());
                telemetry.update();
                this.turret.update();
            }
        }
        this.turret.setPower(0);
        this.turret.update();
        this.turret.stopAndResetEncoder();
        this.turret.update();
        telemetry.addLine("Ready!");
        telemetry.update();
        this.armPosition = 4;
    }


    private void initAsync(){
        drivetrainThread = new AsyncThreaded(() -> {})
                .then(() -> {
                    while (this.opModeInInit() || this.opModeIsActive() && !AsyncThreaded.stopped) updateDrivetrain();
                });
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
    private void turretAutoAimTest() {
        this.updateDrivetrain();
        turretRTP = true;
        turretDesiredAngle = Math.max(0,Math.min(1,(Math.toDegrees(angleWrap((Math.atan2(0 - poseEstimate.getY(), -10 - poseEstimate.getX()) - poseEstimate.getHeading() - Math.toRadians(180))))+90)/180));
        //0 and -10 are the targets. Ill organize it later but basically we gotta make a coordinate for every pole.
        this.targetTurretPosition = turretDesiredAngle;
        this.updateAll();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        while(opModeInInit()){
            while(opModeInInit()){
                AsyncThreaded.stopped = true;
                this.initAll();
                this.leftLinSlide.stopAndResetEncoder();
                resetDepositPosition();
                this.targetPitchPosition = 0.5;
                this.targetTurretPosition = 0.5;
                this.turretRTP = true;
                this.pitchRTP = true;
                waitForStart();
            }
        }
        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0.3) {
                scoringPosition1TurretLeft();
            } else if(gamepad1.right_trigger > 0.3) {
                scoringPosition1TurretRight();
            }
            this.updateDrivetrain();
            turretAutoAimTest();
            this.interact();
            this.updateAll();
        }
        AsyncThreaded.stopped = false;
    }

}