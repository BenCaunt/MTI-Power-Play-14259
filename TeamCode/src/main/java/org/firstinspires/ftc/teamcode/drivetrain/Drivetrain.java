package org.firstinspires.ftc.teamcode.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public interface Drivetrain {
    public Drivetrain setPowerX(double powerX);
    public Drivetrain setPowerY(double powerY);
    public Drivetrain setPowerR(double powerR);
    public Drivetrain setPower(double power, double r);
    public Drivetrain setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior);
    public Drivetrain addPowerX(double powerX);
    public Drivetrain addPowerY(double powerY);
    public Drivetrain addPowerR(double powerR);
    public Drivetrain addPower(double power, double r);
    public double getPowerX();
    public double getPowerY();
    public double getPowerR();
    public void update();
}