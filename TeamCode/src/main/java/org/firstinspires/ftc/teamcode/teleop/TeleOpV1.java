    package org.firstinspires.ftc.teamcode.teleop;

    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DigitalChannel;
    import com.qualcomm.robotcore.hardware.Servo;

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
public class TeleOpV1 extends LinearOpMode {
    //
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
    private double[] linSlidePositions = {0,0.5,0.7};
    private double pitchReset = 0;
    private double turretReset = 0;
    private double pitchLastPosition = 0;
    private double turretLastPosition = 0;
    private double targetPitchPosition;
    private double targetTurretPosition;
    private int step = 0;
    private goBildaTouchDriver pitchTouchSensor;
    private final double[] clawPositions = { 0.0, 1.0 };
    //Calculation variable!
    private double targetLinSlidePosition = this.linSlidePositions[linSlidePosition];
    private double targetFrontArmPositrion = C.frontArmPositions[frontArmPosition];


    //Tests
    private boolean linSlideHigh = true;
    private boolean turretRTP = false;
    private boolean pitchRTP = false;
    private boolean linSlideRTP = true;
    private boolean armOut = false;
    private boolean pitchTS;

    private static Runnable bPressedReset;
    private static Runnable bPressedTakeCone;
    private static Runnable bPressedMoveCone;
    private static Runnable bPressedDumpCone;
    private static Runnable bPressed;

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
    }
    private void initPID() {
        this.targetPitchPosition = this.pitch.getCurrentPosition();
        this.targetTurretPosition = this.pitch.getCurrentPosition();
        this.linSlidePid = new Pid(new Pid.Coefficients(3.2, 1.2, 0.0),
                () -> this.targetLinSlidePosition - this.leftLinSlide.getCurrentPosition(),
                factor -> {
                    this.leftLinSlide.setPower(M.clamp(-factor, -1, 0.76));
                    this.rightLinSlide.setPower(M.clamp(-factor, -1, 0.76));
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
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                })
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                })
                .subscribeEvent(Controller.EventType.DPAD_LEFT, () -> {
                    if(this.armPosition < 10) this.armPosition += 1;
                    moveArm(armPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_RIGHT, () -> {
                    if(this.armPosition > 0) this.armPosition -= 1;
                    moveArm(armPosition);
                })
                .subscribeEvent(Controller.EventType.Y, () -> {
                    this.latchPosition = (this.latchPosition + 1) % C.latchPositions.length;
                    moveLatch(latchPosition);
                })
                .subscribeEvent(Controller.EventType.A, () -> {
                    if(!armOut) {
                        moveArm(armPosition);
                        armOut = true;
                    }
                    else {
                        moveArm(0);
                        armOut = false;
                    }
                });
        this.controller1
                .subscribeEvent(Controller.EventType.RIGHT_BUMPER, () -> {
                    this.depositPosition = (this.depositPosition + 1) % C.depositPositions.length;
                    moveDeposit(depositPosition);
                })
                .subscribeEvent(Controller.EventType.LEFT_BUMPER, () -> {
                    this.clawPosition = (this.clawPosition + 1) % C.clawPositions.length;
                    moveClaw(clawPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_DOWN, () -> {
                    this.frontArmPosition = (this.frontArmPosition + 1) % C.frontArmPositions.length;
                    moveFrontArm(frontArmPosition);
                })
                .subscribeEvent(Controller.EventType.DPAD_UP, () -> {
                    this.linSlideRTP = true;
                    this.linSlidePosition = (this.linSlidePosition + 1) % this.linSlidePositions.length;
                    this.linSlidePid.update();
                })
                .subscribeEvent(Controller.EventType.Y, () -> {
                    this.latchPosition = (this.latchPosition + 1) % C.latchPositions.length;
                    moveLatch(latchPosition);
                })
                .subscribeEvent(Controller.EventType.B, () -> {
                    if(step == 0){
                        intakeOut();
                        resetLinSlide();
                    }
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
        this.bPressedReset = () -> {
            intakeOut();
            resetLinSlide();
            TeleOpV1.bPressed = TeleOpV1.bPressedTakeCone;
        };

        TeleOpV1.bPressedTakeCone = () -> {
            intakeBack();
            TeleOpV1.bPressed = TeleOpV1.bPressedTakeCone;
        };

        TeleOpV1.bPressedMoveCone = () -> {
            linSlideUp();
            TeleOpV1.bPressed = TeleOpV1.bPressedTakeCone;
        };

        TeleOpV1.bPressedDumpCone = () -> {
            dump();
            TeleOpV1.bPressed = TeleOpV1.bPressedTakeCone;
        };
    }

    private void initPosition() {
        this.deposit.setPosition(C.depositPositions[depositPosition]);
        this.claw.setPosition(C.clawPositions[clawPosition]);
        this.latch.setPosition(C.latchPositions[latchPosition]);
        this.leftArm.setPosition(C.armPositions[armPosition]);
        this.frontArm.setPosition(0.8);
        this.rightArm.setPosition(C.armPositions[armPosition]);

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
        if(pitchRTP) //this.pitchPid.update();
        if(turretRTP) this.turretPid.update();
    }
    private void updateDrivetrain() {
        this.drivetrain.update();
    }
    private void updateControllers() {
       this.controller1.update();
       this.controller2.update();
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
        telemetry.update();
    }
    private void updateVariable() {
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.3 -this.pitch.getCurrentPosition()) ,0,1);
        targetFrontArmPositrion = C.frontArmPositions[frontArmPosition] - (0*C.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
    }
    private void updateSensor() {
        pitchTS = this.pitchTouchSensor.check();
    }


    private void updateAll() {
        updateVariable();
        this.updateDrivetrain();
        this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
        this.updateControllers();
        this.updatePid();
        this.updateSensor();
    }
    private void interact(){
        if(gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1){
            this.turretRTP = false;
            this.turret.setPower(-M.clamp(gamepad1.left_trigger-0.1, 0, 0.7) + M.clamp(gamepad1.right_trigger-0.1, 0, 0.7));
        }
        this.drivetrain.addPowerX(-gamepad1.left_stick_x);
        this.drivetrain.addPowerY(-gamepad1.left_stick_y);
        this.drivetrain.addPowerR(gamepad1.right_stick_x);
        this.pitch.setPower(gamepad1.right_stick_y);
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
            telemetry.addLine("Ready! btw veer monke!");
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            this.interact();
            this.updateAll();
        }
    }
    private void moveClaw(int position){
        this.clawPosition = position;
        this.claw.setPosition(C.clawPositions[clawPosition]);
        this.updateServo();
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
        targetFrontArmPositrion = C.frontArmPositions[frontArmPosition] - (0.3*C.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
        this.frontArm.setPosition(targetFrontArmPositrion);
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

    private void intakeOut(){
        frontArmPosition = 1;
        moveFrontArm(frontArmPosition);
        moveArm(armPosition);
        clawPosition = 0;
        moveClaw();
    }
    private void resetPitch(){
        pitchRTP = true;
        this.targetPitchPosition = 0.5;
        this.updateMotor();
    }

    private void greatRest(){
        //frontArm
    }
        private void resetLinSlide(){
        if(depositPosition != 0){
            depositPosition = 0;
            moveDeposit(depositPosition);
            sleep(300);
        }
        linSlidePosition = 0;
        moveLinSlide(linSlidePosition);
        //Add turret center code here later!
    }
        private void linSlideUp(){
        clawPosition = 0;
        moveClaw(clawPosition);
        sleep(500);
        if(linSlideHigh) linSlidePosition = 2;
        else linSlidePosition = 1;
        moveLinSlide(linSlidePosition);
        sleep(250);
        depositPosition = 1;
        moveDeposit(depositPosition);

        if(linSlideHigh) preIntakeMode();
    }
        private void preIntakeMode(){
        frontArm.setPosition(0.5*C.frontArmPositions[frontArmPosition]);
        moveArm(armPosition/2 - 1);
    }
    private void dump(){
        depositPosition = 2;
        moveDeposit(depositPosition);
    }
        private void intakeBack(){
        clawPosition = 1;
        moveClaw(clawPosition);
        //All Sleep function will be replaced by time mangers.
        sleep(250);
        frontArmPosition = 0;
        moveFrontArm(frontArmPosition);
        moveArm(-1);
    }


}