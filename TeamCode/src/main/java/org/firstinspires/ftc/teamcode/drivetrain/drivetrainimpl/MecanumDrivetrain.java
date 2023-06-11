package org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;

public class MecanumDrivetrain implements Drivetrain {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private double powerX = 0.0, powerY = 0.0, powerR = 0.0;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;

    public MecanumDrivetrain(HardwareMap hardwareMap) {
        this.frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        this.frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        this.backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        this.backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        this.frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        this.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        this.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Drivetrain setPowerX(double powerX) { this.powerX = powerX; return this; }
    public Drivetrain setPowerY(double powerY) { this.powerY = powerY; return this; }
    public Drivetrain setPowerR(double powerR) { this.powerR = powerR; return this; }
    public Drivetrain setPower(double power, double r) { this.powerX = power * Math.cos(r); this.powerY = power * Math.sin(r); return this; }

    public Drivetrain setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        this.frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        this.backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        this.backRight.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }

    public Drivetrain addPowerX(double powerX) { this.powerX += powerX; return this; }
    public Drivetrain addPowerY(double powerY) { this.powerY += powerY; return this; }
    public Drivetrain addPowerR(double powerR) { this.powerR += powerR; return this; }
    public Drivetrain addPower(double power, double r) { this.powerX += power * Math.cos(r); this.powerY += power * Math.sin(r); return this; }

    public double getPowerX() { return this.powerX; }
    public double getPowerY() { return this.powerY; }
    public double getPowerR() { return this.powerR; }

    public void update() {
        this.powerX *= C.POWER_X_SCALE;

        double denominator = Math.max(Math.abs(this.powerX) + Math.abs(this.powerY) + Math.abs(this.powerR), 1.0);
        double frontLeftPower = (-this.powerY + this.powerX - this.powerR) / denominator;
        double backLeftPower = (-this.powerY - this.powerX - this.powerR) / denominator;
        double frontRightPower = (-this.powerY - this.powerX + this.powerR) / denominator;
        double backRightPower = (-this.powerY + this.powerX + this.powerR) / denominator;

        this.frontLeft.setPower(frontLeftPower);
        this.backLeft.setPower(backLeftPower);
        this.frontRight.setPower(frontRightPower);
        this.backRight.setPower(backRightPower);

        this.powerX = 0.0;
        this.powerY = 0.0;
        this.powerR = 0.0;
    }
}