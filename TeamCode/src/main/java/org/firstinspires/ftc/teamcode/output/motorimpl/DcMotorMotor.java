package org.firstinspires.ftc.teamcode.output.motorimpl;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.output.Motor;
import org.firstinspires.ftc.teamcode.utils.M;

public class DcMotorMotor implements Motor {
    private DcMotor dcMotor;
    private DcMotor.Direction direction = DcMotor.Direction.FORWARD;
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_TO_POSITION;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
    private double lowerBound = 0.0;
    private double upperBound = 1.0;
    private double position = 0.0;
    private double power = 0.0;

    public DcMotorMotor(DcMotor dcMotor) {
        this.dcMotor = dcMotor;

        dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public DcMotorMotor setLowerBound(double bound) { this.lowerBound = bound; return this; }
    public DcMotorMotor setUpperBound(double bound) { this.upperBound = bound; return this; }

    public DcMotorMotor setDirection(DcMotor.Direction direction) { this.direction = direction; return this; }
    public DcMotorMotor setMode(DcMotor.RunMode runMode) { this.runMode = runMode; return this; }

    public DcMotorMotor setPosition(double position) {
        this.position = position;
        this.runMode = DcMotor.RunMode.RUN_TO_POSITION;
        return this;
    }

    public DcMotorMotor setPower(double power) {
        this.power = power;
        this.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        return this;
    }

    public DcMotorMotor setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.dcMotor.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }

    public DcMotorMotor addPosition(double position) {
        this.position += position;
        this.runMode = DcMotor.RunMode.RUN_TO_POSITION;
        return this;
    }

    public DcMotorMotor addPower(double power) {
        this.power += power;
        this.runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        return this;
    }

    public double getPower() {
        return this.power;
    }

    public double getTargetPosition() {
        return M.normalize(this.dcMotor.getTargetPosition(), this.lowerBound, this.upperBound);
    }

    public double getCurrentPosition() {
        return M.normalize(this.dcMotor.getCurrentPosition(), this.lowerBound, this.upperBound);
    }

    public DcMotor.RunMode getMode() {
        return this.dcMotor.getMode();
    }

    public DcMotorMotor stop() {
        this.position = this.getCurrentPosition();
        this.power = 0.0;
        return this;
    }

    public DcMotorMotor stopAndResetEncoder() {
        this.stop();
        this.dcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        return this;
    }

    public boolean isBusy() { return this.dcMotor.isBusy(); }

    public void update() {
        switch (this.runMode) {
            case RUN_TO_POSITION: {
                double position = M.lerp(this.lowerBound, this.upperBound, this.position);
                if (this.direction != this.dcMotor.getDirection()) position = this.upperBound - (position - this.lowerBound);
                this.dcMotor.setTargetPosition((int) position);
                this.dcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            }
            case RUN_WITHOUT_ENCODER: {
                double power = this.power;
                if (this.direction != this.dcMotor.getDirection()) power = -power;
                this.dcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                this.dcMotor.setPower(power);
                this.power = 0.0;
                this.position = this.getCurrentPosition();
                break;
            }
        }
    }
}