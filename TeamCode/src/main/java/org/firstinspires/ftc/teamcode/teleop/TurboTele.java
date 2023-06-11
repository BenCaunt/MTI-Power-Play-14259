package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.Controller;
import org.firstinspires.ftc.teamcode.input.controllerimpl.GamepadController;
import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;
@Disabled
@TeleOp
public class TurboTele extends LinearOpMode {
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
    private Controller controller1;
    private Controller controller2;

    //Variable
    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private int clawPosition = 0;
    private int latchPosition = 0;
    private int frontArmPosition = 1;
    private int armPosition = 5;
    private int pitchPosition;
    private double[] linSlidePositions = {0,0.5,1};
    private double pitchReset = 0;
    private double turretReset = 0;
    private double pitchLastPosition = 0;
    private double turretLastPosition = 0;
    private double targetPitchPosition;
    private double targetTurretPosition;
    private int step = 0;
    private goBildaTouchDriver pitchTouchSensor;
    private final double[] clawPositions = { 0.0, 1.0};
    //Calculation variable!
    private double targetLinSlidePosition = this.linSlidePositions[linSlidePosition];
    private double targetFrontArmPosition = C.frontArmPositions[frontArmPosition];
    private double targetDepositPosition = C.depositPositions[depositPosition];
    private double[] storedPosition1 = {0.5,0.5}; //Pitch turret
    private int linSlideLevel = 2;
    private double targetTurretPower = 0;
    private int scoringStep = 1;

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

    private void initControllers() {
        this.controller1 = new GamepadController(gamepad1);
        this.controller2 = new GamepadController(gamepad2);
        this.controller2
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                })
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                })
                .subscribeEvent(Controller.EventType.Y, () -> {
                    this.latchPosition = (this.latchPosition + 1) % C.latchPositions.length;
                    moveLatch(latchPosition);
                })
//                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
//                    if(this.pitch.getCurrentPosition() > 0.1) movePitch(targetPitchPosition - 0.05);
//                })
//                .subscribeEvent(Controller.EventType.DPAD_UP, () -> {
//                    if(this.pitch.getCurrentPosition() < 0.9) movePitch(targetPitchPosition + 0.05);
//                })
                .subscribeEvent(Controller.EventType.A, () -> {
                    resetDeposit();
                    movePitch(0.5);
                    this.updateMotor();
                })
                .subscribeEvent(Controller.EventType.DPAD_UP, () -> {
                    scoringPosition1();
                })
                .subscribeEvent(Controller.EventType.DPAD_LEFT, () -> {
                    scoringPosition2();
                })
                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
                    scoringPosition3();
                })
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                    this.depositPosition = (this.depositPosition + 1) % C.depositPositions.length;
                    moveDeposit(depositPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
                    linSlideReset();
                });

        this.controller1
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                    this.depositPosition = (this.depositPosition + 1) % C.depositPositions.length;
                    moveDeposit(depositPosition);
                })
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                    this.clawPosition = (this.clawPosition + 1) % C.clawPositions.length;
                    moveClaw(clawPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_LEFT, () -> {
                    if(this.armPosition < 9) this.armPosition += 1;
                    moveArm(armPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
                    if(this.armPosition > 0) this.armPosition -= 1;
                    moveArm(armPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
                    this.frontArmPosition = (this.frontArmPosition + 1) % C.frontArmPositions.length;
                    moveFrontArm(frontArmPosition);
                })
//                .subscribeEvent(Controller.EventType.DPAD_UP, () -> {
//                    this.linSlideRTP = true;
//                    this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
//                    this.linSlidePid.update();
//                })
//                .subscribeEvent(Controller.EventType.Y, () -> {
//                    this.latchPosition = (this.latchPosition + 1) % C.latchPositions.length;
//                    moveLatch(latchPosition);
//                })
//                .subscribeEvent(Controller.EventType.B, () -> {
//                    if(step == 0){
//                        intakeOut();
//                        step = 1;
//                    }else if(step == 1){
//                        intakeBack();
//                        step = 2;
//                    }else{
//                        moveClaw(0);
//                        sleep(100);
//                        preIntakeMode();
//                        step = 0;
//                    }
//                })
//                .subscribeEvent(Controller.EventType.Y, () -> {
//                    if(scoringStep == 1) {
//                        goToLastPosition();
//                        scoringStep++;
//                    }else if(scoringStep == 2){
//                        this.storeCurrentPosition();
//                        moveLatch(0);
//                        scoringStep ++;
//                    }else{
//                        linSlideReset();
//                        this.scoringStep = 1;
//                    }
//                })
                .subscribeEvent(Controller.EventType.B, () -> {
                    if(step == 0){
                        linSlideReset();
                        this.scoringStep = 1;
                        intakeOut();
                        step = 1;
                    }else if(step == 1){
                        intakeBack();
                        step = 2;
                    }else if(step == 2) {
                        moveClaw(0);
                        sleep(100);
                        preIntakeMode();
                        step++;
                    }else if(step == 3){
                        goToLastPosition();
                        step++;
                    }else
                    {
                        this.storeCurrentPosition();
                        moveLatch(0);
                        step = 0;
                    }
                })
                .subscribeEvent(Controller.EventType.X, () -> {
                    this.resetDeposit();
                    this.scoringStep = 1;
                })
                .subscribeEvent(Controller.EventType.RIGHT_STICK_BUTTON, () -> {
                    this.storeCurrentPosition();
                })

                .subscribeEvent(Controller.EventType.A, () -> {
                    if(!armOut) {
                        moveArm(armPosition);
                        armOut = true;
                    }
                    else {
                        this.leftArm.setPosition(0);
                        this.rightArm.setPosition(0);
                        armOut = false;
                    }
                });
    }

    private void interact(){
//        if(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1 || gamepad2.left_trigger > 0.1 || gamepad2.right_trigger > 0.1){
//            this.turretRTP = false;
//            this.turret.setPower(-M.clamp(gamepad1.left_trigger-0.1, 0, 0.7) + M.clamp(gamepad1.right_trigger-0.1, 0, 0.7));
//        }
        if(targetTurretPower > 0.05 || targetTurretPower < -0.05){
            this.turretRTP = false;
            this.turret.setPower(targetTurretPower);
        }
        this.drivetrain.addPowerX(gamepad1.left_stick_x);
        this.drivetrain.addPowerY(-gamepad1.left_stick_y);
        this.drivetrain.addPowerR(gamepad1.right_stick_x);
//        if(this.pitch.getCurrentPosition() > 0.1 || this.pitch.getCurrentPosition() <0.9 && gamepad1.right_stick_y > 0.1 || -gamepad1.right_stick_y > 0.1){
//            this.pitchRTP = false;
//            this.pitch.setPower(0.5*gamepad1.right_stick_y);
//        }
        if(Math.abs(gamepad2.right_stick_y) > 0.1){
            this.linSlideRTP = false;
            this.leftLinSlide.setPower(gamepad2.right_stick_y);
            this.rightLinSlide.setPower(gamepad2.right_stick_y);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            this.initAll();
            this.leftLinSlide.stopAndResetEncoder();
            this.turret.stopAndResetEncoder();
            updateSensor();
            if(pitchTS == false) {
                while (pitchTS == false) {
                    updateSensor();
                    telemetry.addLine(" RESET THE PITCH POSITION !!!");
                    telemetry.addData("Pitch Touch Sensor", pitchTS);
                    telemetry.update();
                    this.pitch.setPower(0.2);
                    this.pitch.update();
                }
            }
            this.pitch.setPower(0);
            this.pitch.update();
            this.pitch.stopAndResetEncoder();
            this.pitch.update();
            if(turretSensorTouched == false) {
                while (turretSensorTouched == false) {
                    updateSensor();
                    telemetry.addLine(" RESET THE TURRET POSITION !!!");
                    telemetry.update();
                    this.turret.setPower(0.35);
                    this.turret.update();
                }
            }
            this.turret.setPower(0);
            this.turret.update();
            this.turret.stopAndResetEncoder();
            this.turret.update();
            telemetry.addLine("Ready! btw veer monke!");
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            this.interact();
            this.updateAll();
        }
    }
    private void initPosition() {
        this.deposit.setPosition(C.depositPositions[depositPosition]);
        this.claw.setPosition(C.clawPositions[clawPosition]);
        this.latch.setPosition(C.latchPositions[latchPosition]);
        this.leftArm.setPosition(0);
        this.frontArm.setPosition(0.8);
        this.rightArm.setPosition(0);
    }
    private void updateTelemetry() {
        telemetry.addData("this.leftLinSlide.getPower()", String.format("%.2f", this.leftLinSlide.getPower()));
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
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.4 - 1*this.targetPitchPosition) ,0,1);
        targetFrontArmPosition = C.frontArmPositions[frontArmPosition];
        targetTurretPower = 0.7*(-gamepad1.left_trigger + gamepad1.right_trigger - 0.5*gamepad2.left_trigger + 0.5*gamepad2. right_trigger);
    }
    private void updateSensor() {
        pitchTS = this.pitchTouchSensor.check();
        turretSensorTouched = this.turretSensor.isPressed();
    }
    private void updateAll() {
        this.updateVariable();
        this.updateDrivetrain();
        this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
        this.updateControllers();
        this.updatePid();
        this.updateSensor();
    }
    private void moveClaw(int position){
        this.clawPosition = position;
        this.claw.setPosition(C.clawPositions[clawPosition]);
        this.updateServo();
    }
    private void movePitch(){
        this.targetPitchPosition = C.pitchPositions[this.pitchPosition];

    }
    private void movePitch(double position){
        this.targetPitchPosition = M.clamp(position , 0.1, 1);
    }
    private void moveDeposit(int position){
        this.depositPosition = position;
        this.deposit.setPosition(C.depositPositions[depositPosition]);
        this.updateServo();
    }
    private void moveLatch(int position){
        this.latchPosition = position;
        this.latch.setPosition(C.latchPositions[latchPosition]);
        this.updateServo();
    }
    private void moveFrontArm(int position){
        this.frontArmPosition = position;
        targetFrontArmPosition = C.frontArmPositions[frontArmPosition] - (0.1*C.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
        this.frontArm.setPosition(targetFrontArmPosition);
        this.updateServo();
    }
    private void moveArm(int position){
        this.armPosition = position;
        this.leftArm.setPosition(C.armPositions[armPosition]);
        this.rightArm.setPosition(C.armPositions[armPosition]);
        this.updateServo();
    }
    private void moveLinSlide(int position){
        linSlideRTP = true;
        this.linSlidePosition = position;
        this.updateMotor();
    }
    private void moveClaw(){
        this.claw.setPosition(this.clawPositions[clawPosition]);
    }
    private void moveTurret(double position){
            this.targetTurretPosition = position;
    }
    private void intakeOut(){
        frontArmPosition = 0;
        moveFrontArm(frontArmPosition);
        moveArm(armPosition);
        clawPosition = 0;
        moveClaw();
    }
    private void resetDeposit(){
        turretRTP = true;
        moveTurret(0.5);
        linSlideReset();
    }
    private void dump(){
        depositPosition = 2;
        this.deposit.setPosition(depositPosition - 0.1* (1-this.targetPitchPosition));
    }
    private void storeCurrentPosition(){
        this.storedPosition1[0] = this.pitch.getCurrentPosition();
        this.storedPosition1[1] = this.turret.getCurrentPosition();
    }
    private void goToLastPosition(){
            pitchRTP = true;
            turretRTP = true;
            movePitch(this.storedPosition1[0]);
            moveTurret(this.storedPosition1[1]);
            moveLatch(1);
            moveLinSlide(linSlideLevel);
            this.dump();

    }
    private void scoringPosition1(){
            pitchRTP = true;
            linSlideRTP = true;
            moveLatch(1);
            scorePos1 = true;
            this.dump();
            movePitch(0.95);
            moveLinSlide(linSlideLevel);
    }
    private void scoringPosition2() {
        if (!scorePos2) {
            pitchRTP = true;
            linSlideRTP = true;
            moveLatch(1);
            scorePos2 = true;
            moveLatch(1);
            this.dump();
            movePitch(0.6);
            moveLinSlide(linSlideLevel);
        } else {
            scorePos2 = false;
            moveLatch(0);
        }
    }
    private void scoringPosition3() {
        if (!scorePos3) {
            preIntakeMode();
            pitchRTP = true;
            linSlideRTP = true;
            moveLatch(1);
            scorePos3 = true;
            moveLatch(1);
            this.depositPosition = 1;
            this.deposit.setPosition(0.9);
            movePitch(0.4);
            moveLinSlide(linSlideLevel);
        } else {
            scorePos3 = false;
            moveLatch(0);
        }
    }
    private void preIntakeMode(){
        this.frontArm.setPosition(0.7);
    }
    private void greatReset(){
        scorePos1 = false;
        scorePos2 = false;
        scorePos3 = false;
        this.frontArm.setPosition(0.5);
        sleep(100);
        moveDeposit(0);
        moveLinSlide(0);
        moveLatch(0);
        movePitch(0.5);
        this.frontArm.setPosition(0.8);
    }
    private void linSlideReset(){
        scorePos1 = false;
        scorePos2 = false;
        scorePos3 = false;
        this.frontArm.setPosition(0.5);
        sleep(100);
        moveDeposit(0);
        moveLinSlide(0);
        moveLatch(0);
        movePitch(0.5);
    }
    private void intakeBack(){
        clawPosition = 1;
        moveClaw(clawPosition);
        //All Sleep function will be replaced by time mangers.
        sleep(250);
        frontArmPosition = 1;
        moveFrontArm(frontArmPosition);
        this.leftArm.setPosition(0);
        this.rightArm.setPosition(0);
        this.updateServo();
    }
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
                    this.leftLinSlide.setPower(M.clamp(-factor, -1, 0.55));
                    this.rightLinSlide.setPower(M.clamp(-factor, -1, 0.55));
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
    private void initAll() {
        this.initMotor();
        this.initServo();
        this.initDrivetrain();
        this.initPID();
        this.initPosition();
        this.initControllers();
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
    private void updatePid() {
        if(linSlideRTP) this.linSlidePid.update();
        if(pitchRTP) this.pitchPid.update();
        if(turretRTP) this.turretPid.update();
    }
    private void updateDrivetrain() {
        this.drivetrain.update();
    }
    private void updateControllers() {
        this.controller1.update();
        this.controller2.update();
    }




}