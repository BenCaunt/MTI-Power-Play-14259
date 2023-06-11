package org.firstinspires.ftc.teamcode.localization.localizationimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.components.DriveComponent;
import org.firstinspires.ftc.teamcode.input.Encoder;
import org.firstinspires.ftc.teamcode.input.encoderimpl.DcMotorExEncoder;
import org.firstinspires.ftc.teamcode.localization.Localization;

public class ThreeWheelLocalization implements Localization {
    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder centerEncoder;
    private double x = DriveComponent.INIT_X, y = DriveComponent.INIT_Y, r = DriveComponent.INIT_R;

    public ThreeWheelLocalization(HardwareMap hardwareMap) {
        this.leftEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
        this.rightEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "FrontRight"));
        this.centerEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "BackRight"));

        this.leftEncoder.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public double getX() { return this.x; }
    public double getY() { return this.y; }
    public double getR() { return this.r; }

    public void update() {
        this.leftEncoder.update();
        this.rightEncoder.update();
        this.centerEncoder.update();

        double rDelta = Math.asin((this.leftEncoder.getDx() - this.rightEncoder.getDx()) * C.PARA_SCALE / (C.LEFT_OFFSET - C.RIGHT_OFFSET));
        double leftDelta = this.leftEncoder.getDx() * C.PARA_SCALE - C.LEFT_OFFSET * rDelta;
        double rightDelta = this.rightEncoder.getDx() * C.PARA_SCALE - C.RIGHT_OFFSET * rDelta;

        double paraDelta = (leftDelta + rightDelta) / 2.0;
        double perpDelta = this.centerEncoder.getDx() * C.PERP_SCALE + C.CENTER_OFFSET * rDelta;
        double xDelta = perpDelta * Math.cos(this.r) + paraDelta * Math.sin(this.r);
        double yDelta = perpDelta * Math.sin(this.r) - paraDelta * Math.cos(this.r);

        this.x += xDelta;
        this.y += yDelta;
        this.r += rDelta;
    }
}