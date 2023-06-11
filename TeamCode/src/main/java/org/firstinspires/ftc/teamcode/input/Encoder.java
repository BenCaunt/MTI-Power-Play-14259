package org.firstinspires.ftc.teamcode.input;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface Encoder {
    public double getX();
    public double getDx();
    public double getDxDt();
    public Encoder setDirection(DcMotorEx.Direction direction);
    public void update();
}