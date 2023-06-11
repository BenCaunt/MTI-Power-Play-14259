package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.Async;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;
import org.firstinspires.ftc.teamcode.input.Controller;
import org.firstinspires.ftc.teamcode.input.controllerimpl.GamepadController;
import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;

@TeleOp
public class DemoSystemOpMode extends LinearOpMode {
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
    private AsyncThreaded drivetrainThread;

    //Variable
    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private int clawPosition = 0;
    private int latchPosition = 0;
    private int frontArmPosition = 1;
    private int armPosition = 0;
    private int pitchPosition;
    private double[] linSlidePositions = {0,0.45,1};
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
    private double lastTurretPosition = 0.5;

    private boolean clawOpen = true;
    private boolean latchEngaged = false;
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
    private boolean turretSensorTouched;
    private boolean turretMode = false;
    private int scorePos = 1;

    private int intakeStep = 1;
    private int depositStep = 1;
    private int cycleStep = 1;

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

    private void initControllers() {
        this.controller1 = new GamepadController(gamepad1);
        this.controller2 = new GamepadController(gamepad2);
        this.controller2
//                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
//                    scorePos = 4;
//                })
                .subscribeEvent(Controller.EventType.A, () -> {

                })
                .subscribeEvent(Controller.EventType.X, () -> {
                    this.turretRTP = true;
                    this.targetTurretPosition = 0.5;
                })
                .subscribeEvent(Controller.EventType.RIGHT_STICK_BUTTON, () -> {
                    linSlideHigh = false;
                })
                .subscribeEvent(Controller.EventType.LEFT_STICK_BUTTON, () -> {
                    this.pitchRTP = true;
//                    this.lastTurretPosition = this.turret.getCurrentPosition();
                    this.greatReset();
                    if(Math.abs(this.turret.getCurrentPosition() - 0.5) > 0.25){
                        this.turretRTP = true;
                        this.targetTurretPosition = 0.5;
                    }
                    intakeBack();
                })
                .subscribeEvent(Controller.EventType.Y, () -> {
                    this.targetFrontArmPosition += 0.05;
                })
                .subscribeEvent(Controller.EventType.B, () -> {
                    switch (cycleStep) {
                        case 1:
                            linSlideReset();
                            this.intakeOut();
                            this.cycleStep++;
                            break;
                        case 2:
                            if(Math.abs(this.turret.getCurrentPosition() - 0.5) > 0.3){
                                this.turretRTP = true;
                                this.targetTurretPosition = 0.5;
                            }
                            this.intakeBack();
                            this.cycleStep++;
                            break;
                        case 3:
                            this.clawOpen = true;
                            this.updatePosition();
                            this.updateServo();
                            sleep(300);
                            this.preIntakeMode();
                            cycleStep++;
                            break;
                        case 4:
                            switch(scorePos){
                                case 1:
                                    scoringPosition1();
                                    break;
                                case 2:
                                    scoringPosition2();
                                    break;
                                case 3:
                                    scoringPosition3();
                                    break;
                                case 4:
                                    scoringPosition4();
                                    break;
                                case 5:
                                    scoringPosition5();
                                    break;
                                case 6:
                                    scoringPosition6();
                                    break;
                            }
                            cycleStep++;
                            break;
                        default:
                            latchEngaged = false;
                            cycleStep = 1;
                            break;
                    }
                })
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                    if(this.armPosition > 1) this.armPosition--;
                    moveArm(armPosition);
                })
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                    if(this.armPosition < 9) this.armPosition++;
                    moveArm(armPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_UP, () -> {
                    scorePos = 1;
                    this.scoringPosition1();
                })
                .subscribeEvent(Controller.EventType.DPAD_LEFT, () -> {
                    if(targetTurretPosition < 0.8){
                        this.targetTurretPosition += 0.1;
                    }
                })
                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
                    if(targetTurretPosition > 0.2){
                        this.targetTurretPosition -= 0.1;
                    }
                })
                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
                    this.greatReset();
                });

        this.controller1
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                    this.latchEngaged = !this.latchEngaged;
                    if(this.targetDepositPosition > 0.7)this.lastTurretPosition = this.turret.getCurrentPosition();
                })
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                    this.clawOpen = !this.clawOpen;
                })
                .subscribeEvent(Controller.EventType.Y, () -> {
                    this.targetFrontArmPosition = 0.47;
                    intakeStep = 1;
                })
                .subscribeEvent(Controller.EventType.DPAD_UP, () -> {
                    scorePos = 1;
                    this.scoringPosition1();
                })
//                .subscribeEvent(Controller.EventType.DPAD_LEFT, () -> {
//                    scorePos = 2;
//                    this.scoringPosition2();
//                })
//                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
//                    scorePos = 3;
//                    this.scoringPosition3();
//                })
                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
                    this.midPole();
                })
                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
                    this.frontArmPosition = (this.frontArmPosition + 1) % C.frontArmPositions.length;
                    moveFrontArm();
                })
//                .subscribeEvent(Controller.EventType.RIGHT_STICK_BUTTON, () -> {
//                    linSlideHigh = true;
//                })
//                .subscribeEvent(Controller.EventType.LEFT_STICK_BUTTON, () -> {
//                    linSlideHigh = false;
//                })
                .subscribeEvent(Controller.EventType.B, () -> {
                    switch (intakeStep) {
                        case 1:
                            this.intakeOut();
                            this.intakeStep++;
                            break;
                        case 2:
                            this.intakeBack();
                            this.intakeStep++;
                            break;
                        default:
                            this.clawOpen = true;
                            this.updatePosition();
                            this.updateServo();
                            sleep(300);
                            this.preIntakeMode();
                            intakeStep = 1;
                    }
                })
                .subscribeEvent(Controller.EventType.A, () -> {
                    if(!armOut) {
                        moveArm(armPosition);
                        armOut = true;
                    }
                    else {
                        moveArm(-1);
                        armOut = false;
                    }
                })
                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
                    this.pitchRTP = true;
//                    this.lastTurretPosition = this.turret.getCurrentPosition();
                    this.greatReset();
                    if(Math.abs(this.turret.getCurrentPosition() - 0.5) > 0.25){
                        this.turretRTP = true;
                        this.targetTurretPosition = 0.5;
                    }
                });
    }
    private void initAsync(){
        drivetrainThread = new AsyncThreaded(() -> {})
                .then(() -> {
                    while (this.opModeInInit() || this.opModeIsActive() && !AsyncThreaded.stopped) updateDrivetrain();
                });
    }

    private void initPosition() {
        moveArm(0);
        this.targetFrontArmPosition = 0.9;
    }
    private void initAll() {
        this.initMotor();
        this.initServo();
        this.initDrivetrain();
        this.initPID();
        this.initPosition();
        this.initControllers();
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
    private void updateDrivetrain() {
        this.drivetrain.addPowerX(gamepad1.left_stick_x);
        this.drivetrain.addPowerY(gamepad1.left_stick_y);
        this.drivetrain.addPowerR(-gamepad1.right_stick_x * 0.8);
        this.drivetrain.update();
    }
    private void updateControllers() {
        this.controller1.update();
        this.controller2.update();
    }
    private void updateTelemetry() {
        telemetry.addLine("Demo Tele-Op: \n up and down d pad for slide up and down\n left and Right to move Turret Left and right\n");

        telemetry.addData("Linear Slide Current Value:" , this.leftLinSlide.getCurrentPosition());
        telemetry.addData("Linear Slide Target Position:" , this.targetLinSlidePosition);

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
        this.updateSensor();
        this.updateControllers();
        this.updatePosition();
        this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
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

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeInInit()){
            AsyncThreaded.stopped = true;
            this.initAll();
            this.leftLinSlide.stopAndResetEncoder();
            resetDepositPosition();
            this.targetPitchPosition = 0.5;
            this.targetTurretPosition = 0.5;
            this.turretRTP = true;
            this.pitchRTP = true;
            this.drivetrainThread.run();
            waitForStart();
        }
        waitForStart();
        while (opModeIsActive()) {
            this.interact();
            this.updateAll();
        }
        AsyncThreaded.stopped = false;
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
        this.targetDepositPosition = (C.depositPositions[depositPosition]) - 0.0*(C.depositPositions[depositPosition] * (M.clamp(this.targetPitchPosition, 0, 0.8)));
        this.updatePosition();
        this.updateServo();
    }
    private void moveFrontArm(){
        targetFrontArmPosition = this.frontArmPositions[frontArmPosition] - (0.1*this.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
        this.updatePosition();
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
    private void preIntakeMode(){
        this.targetFrontArmPosition = 0.6;
    }
    private void greatReset(){
        this.linSlideRTP = true;
        this.depositPosition = 0;
        moveDeposit();
        this.linSlidePosition = 0;
        latchEngaged = false;
        movePitch(0.5);
    }
    private void linSlideReset(){
        if(targetFrontArmPosition > 0.8) {
            targetFrontArmPosition = 0.7;
            sleep(100);
        }
        this.depositPosition = 0;
        moveDeposit();
        if(latchEngaged)this.latchEngaged = false;
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
        telemetry.addLine("Ready!");
        telemetry.update();
        this.armPosition = 4;
    }
}