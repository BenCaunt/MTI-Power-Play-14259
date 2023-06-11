package org.firstinspires.ftc.teamcode.input.encoderimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.input.Encoder;

public class DcMotorExEncoder implements Encoder {
    private DcMotorEx dcMotorEx;
    private DcMotorEx.Direction direction = DcMotorEx.Direction.FORWARD;
    private double x = 0.0;
    private double dx = 0.0;
    private double dxDt = 0.0;

    public DcMotorExEncoder(DcMotorEx dcMotorEx) {
        this.dcMotorEx = dcMotorEx;

        DcMotorEx.RunMode pRunMode = dcMotorEx.getMode();
        dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorEx.setMode(pRunMode);
    }

    public Encoder setDirection(DcMotorEx.Direction direction) { this.direction = direction; return this; }

    public double getX() { return this.x; }
    public double getDx() { return this.dx; }
    public double getDxDt() { return this.dxDt; }

    public void update() {
        double x = this.dcMotorEx.getCurrentPosition() / C.INCHES_TO_TICKS;
        if (this.direction != this.dcMotorEx.getDirection()) x = -x;

        double dx = x - this.x;
        double dxDt = this.dcMotorEx.getVelocity() / C.INCHES_TO_TICKS;

        this.x = x;
        this.dx = dx;
        this.dxDt = dxDt;
    }
}