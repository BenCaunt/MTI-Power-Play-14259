package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface Drive {
    public Drive setTargetX(double x);
    public Drive setTargetY(double y);
    public Drive setTargetR(double r);
    public Drive addTargetX(double x);
    public Drive addTargetY(double y);
    public Drive addTargetR(double r);
    public Drive setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior);
    public double getCurrentX();
    public double getCurrentY();
    public double getCurrentR();
    public double getErrorX();
    public double getErrorY();
    public double getErrorR();
    public boolean isBusy();
    public void update();
}