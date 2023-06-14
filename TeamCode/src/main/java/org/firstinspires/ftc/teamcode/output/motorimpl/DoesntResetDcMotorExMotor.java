package org.firstinspires.ftc.teamcode.output.motorimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.output.Motor;
import org.firstinspires.ftc.teamcode.utils.M;

public class DoesntResetDcMotorExMotor implements Motor {
    private DcMotorEx dcMotorEx;
    private DcMotorEx.Direction direction = DcMotorEx.Direction.FORWARD;
    private DcMotorEx.RunMode runMode = DcMotorEx.RunMode.RUN_TO_POSITION;
    private DcMotorEx.ZeroPowerBehavior zeroPowerBehavior = DcMotorEx.ZeroPowerBehavior.FLOAT;
    private double lowerBound = 0.0;
    private double upperBound = 1.0;
    private double position = 0.0;
    private double power = 0.0;

    public DoesntResetDcMotorExMotor(DcMotorEx dcMotorEx) {
        this.dcMotorEx = dcMotorEx;

//        dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //who the fuck did this
    }

    public DoesntResetDcMotorExMotor setLowerBound(double bound) { this.lowerBound = bound; return this; }
    public DoesntResetDcMotorExMotor setUpperBound(double bound) { this.upperBound = bound; return this; }

    public DoesntResetDcMotorExMotor setDirection(DcMotorEx.Direction direction) { this.direction = direction; return this; }
    public DoesntResetDcMotorExMotor setMode(DcMotorEx.RunMode runMode) { this.runMode = runMode; return this; }

    public DoesntResetDcMotorExMotor setPosition(double position) {
        this.position = position;
        this.runMode = DcMotorEx.RunMode.RUN_TO_POSITION;
        return this;
    }

    public DoesntResetDcMotorExMotor setPower(double power) {
        this.power = power;
        this.runMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        return this;
    }

    public DoesntResetDcMotorExMotor setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.dcMotorEx.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }

    public DoesntResetDcMotorExMotor addPosition(double position) {
        this.position += position;
        this.runMode = DcMotorEx.RunMode.RUN_TO_POSITION;
        return this;
    }

    public DoesntResetDcMotorExMotor addPower(double power) {
        this.power += power;
        this.runMode = DcMotorEx.RunMode.RUN_WITHOUT_ENCODER;
        return this;
    }

    public double getPower() {
        return this.power;
    }

    public double getTargetPosition() {
        return M.normalize(this.dcMotorEx.getTargetPosition(), this.lowerBound, this.upperBound);
    }

    public double getCurrentPosition() {
        return M.normalize(this.dcMotorEx.getCurrentPosition(), this.lowerBound, this.upperBound);
    }

    public DcMotorEx.RunMode getMode() {
        return this.dcMotorEx.getMode();
    }

    public DoesntResetDcMotorExMotor stop() {
        this.position = this.getCurrentPosition();
        this.power = 0.0;
        return this;
    }

    public DoesntResetDcMotorExMotor stopAndResetEncoder() {
        this.stop();
        this.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        return this;
    }

    public boolean isBusy() { return this.dcMotorEx.isBusy(); }

    public void update() {
        switch (this.runMode) {
            case RUN_TO_POSITION: {
                double position = M.lerp(this.lowerBound, this.upperBound, this.position);
                if (this.direction != this.dcMotorEx.getDirection()) position = this.upperBound - (position - this.lowerBound);
                this.dcMotorEx.setTargetPosition((int) position);
                this.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                break;
            }
            case RUN_WITHOUT_ENCODER: {
                double power = this.power;
                if (this.direction != this.dcMotorEx.getDirection()) power = -power;
                this.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                this.dcMotorEx.setPower(power);
                this.power = 0.0;
                this.position = this.getCurrentPosition();
                break;
            }
        }
    }
}