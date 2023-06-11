package org.firstinspires.ftc.teamcode.output;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
TODO:

Child interface for DcMotor and Servo since they're
pretty different
*/

public interface Motor {
    public Motor setLowerBound(double bound);
    public Motor setUpperBound(double bound);
    public Motor setDirection(DcMotor.Direction direction);
    public Motor setPosition(double position);
    public Motor addPosition(double position);
    public void update();
}