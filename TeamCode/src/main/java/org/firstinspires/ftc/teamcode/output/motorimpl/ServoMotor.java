package org.firstinspires.ftc.teamcode.output.motorimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.output.Motor;
import org.firstinspires.ftc.teamcode.utils.M;

public class  ServoMotor implements Motor {
    private Servo servo;
    private Servo.Direction direction;
    private double lowerBound = 0.0;
    private double upperBound = 1.0;
    private double position = 0.0;

    public ServoMotor(Servo dcMotorEx) {
        this.servo = dcMotorEx;
    }

    public ServoMotor setLowerBound(double bound) { this.lowerBound = bound; return this; }
    public ServoMotor setUpperBound(double bound) { this.upperBound = bound; return this; }

    public ServoMotor setDirection(DcMotorEx.Direction direction) {
        switch (direction) {
            case FORWARD:
                this.direction = Servo.Direction.FORWARD;
                break;
            case REVERSE:
                this.direction = Servo.Direction.REVERSE;
                break;
        }
        return this;
    }

    public ServoMotor setPosition(double position) {
        this.position = position;
        return this;
    }

    public double getPosition(){
        return this.position;
    }

    public ServoMotor addPosition(double position) {
        this.position += position;
        return this;
    }

    public void update() {
        this.servo.setPosition(M.lerp(this.lowerBound, this.upperBound, this.position));
    }
}