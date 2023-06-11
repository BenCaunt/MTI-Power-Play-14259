package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorMotor;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.M;

public class LinearSlideComponent implements Component {
    private static final double INCHES_TO_TICKS = -1;
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = -1250;
    private static final double INIT_INCHES = 0;

    private Component parent;
    private DcMotorMotor leftMotor;
    private DcMotorMotor rightMotor;

    public LinearSlideComponent(Component parent, HardwareMap hardwareMap) {
        this.parent = parent;
        this.leftMotor = new DcMotorMotor(hardwareMap.get(DcMotor.class, "leftLinSlide"))
                .setLowerBound(LinearSlideComponent.LOWER_BOUND)
                .setUpperBound(LinearSlideComponent.UPPER_BOUND);
        this.rightMotor = new DcMotorMotor(hardwareMap.get(DcMotor.class, "rightLinSlide"))
                .setLowerBound(LinearSlideComponent.LOWER_BOUND)
                .setUpperBound(LinearSlideComponent.UPPER_BOUND);
    }

    public LinearSlideComponent setPosition(double position) {
        this.leftMotor.setPosition(position);
        this.rightMotor.setPosition(position);
        return this;
    }

    public LinearSlideComponent setInches(double inches) {
        this.setPosition(M.normalize((inches - LinearSlideComponent.INIT_INCHES) * LinearSlideComponent.INCHES_TO_TICKS, LinearSlideComponent.LOWER_BOUND, LinearSlideComponent.UPPER_BOUND));
        return this;
    }

    public LinearSlideComponent setPower(double power) {
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
        return this;
    }

    public LinearSlideComponent addPosition(double position) {
        this.leftMotor.addPosition(position);
        this.rightMotor.addPosition(position);
        return this;
    }

    public LinearSlideComponent addPower(double power) {
        this.leftMotor.addPower(power);
        this.rightMotor.addPower(power);
        return this;
    }

    public DcMotorEx.RunMode getMode() {
        return this.leftMotor.getMode();
    }

    public LinearSlideComponent stop() {
        this.leftMotor.stop();
        this.rightMotor.stop();
        return this;
    }

    public LinearSlideComponent stopAndResetEncoder() {
        this.leftMotor.stopAndResetEncoder();
        this.rightMotor.stopAndResetEncoder();
        return this;
    }

    public boolean isBusy() { return this.leftMotor.isBusy() || this.rightMotor.isBusy(); }

    public double getCurrentPosition() {
        return this.leftMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return this.leftMotor.getTargetPosition();
    }

    public double getCurrentInches() {
        return M.lerp(LinearSlideComponent.LOWER_BOUND, LinearSlideComponent.UPPER_BOUND, this.getCurrentPosition()) / LinearSlideComponent.INCHES_TO_TICKS + LinearSlideComponent.INIT_INCHES;
    }

    public double getTargetInches() {
        return M.lerp(LinearSlideComponent.LOWER_BOUND, LinearSlideComponent.UPPER_BOUND, this.getTargetPosition()) / LinearSlideComponent.INCHES_TO_TICKS + LinearSlideComponent.INIT_INCHES;
    }

    public Component getParent() {
        return this.parent;
    }

    public Vector3 transformPosition(Vector3 position) {
        return this.parent.transformPosition(position.rotatePitch(this.getCurrentInches()));
    }

    public Vector3 transformRotation(Vector3 rotation) {
        return this.parent.transformPosition(rotation);
    }

    public void update() {
        this.leftMotor.update();
        this.rightMotor.update();
    }
}
