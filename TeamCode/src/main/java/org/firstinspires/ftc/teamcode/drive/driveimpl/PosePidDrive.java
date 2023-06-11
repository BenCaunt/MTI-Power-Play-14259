package org.firstinspires.ftc.teamcode.drive.driveimpl;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.components.DriveComponent;
import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.input.Timer;
import org.firstinspires.ftc.teamcode.localization.Localization;
import org.firstinspires.ftc.teamcode.localization.localizationimpl.ThreeWheelLocalization;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.utils.M;

public class PosePidDrive implements Drive {
    private Drivetrain drivetrain;
    private Localization localization;
    private Pid xPid, yPid, rPid;
    private Timer timer;
    private double x = DriveComponent.INIT_X, y = DriveComponent.INIT_Y, r = DriveComponent.INIT_R;
    private double busyEt = 0.0;

    public PosePidDrive(HardwareMap hardwareMap) {
        this.drivetrain = new MecanumDrivetrain(hardwareMap);
        this.localization = new ThreeWheelLocalization(hardwareMap);
        this.xPid = new Pid(C.xPidCoefficients,
                () -> this.getErrorX(),
                factor -> this.drivetrain.addPower(factor, this.getCurrentR()));
        this.yPid = new Pid(C.yPidCoefficients,
                () -> this.getErrorY(),
                factor -> this.drivetrain.addPower(factor, this.getCurrentR() - Math.PI / 2.0));
        this.rPid = new Pid(C.rPidCoefficients,
                () -> this.getErrorR(),
                factor -> this.drivetrain.addPowerR(factor));
        this.timer = new Timer();
    }

    public Drive setTargetX(double x) { this.x = x; this.busyEt = 0.0; return this; }
    public Drive setTargetY(double y) { this.y = y; this.busyEt = 0.0; return this; }
    public Drive setTargetR(double r) { this.r = r; this.busyEt = 0.0; return this; }

    public Drive addTargetX(double x) { this.x += x; this.busyEt = 0.0; return this; }
    public Drive addTargetY(double y) { this.y += y; this.busyEt = 0.0; return this; }
    public Drive addTargetR(double r) { this.r += r; this.busyEt = 0.0; return this; }

    public Drive setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        this.drivetrain.setZeroPowerBehavior(zeroPowerBehavior);
        return this;
    }

    public double getCurrentX() { return this.localization.getX(); }
    public double getCurrentY() { return this.localization.getY(); }
    public double getCurrentR() { return this.localization.getR(); }

    public double getErrorX() { return this.x - this.localization.getX(); }
    public double getErrorY() { return this.y - this.localization.getY(); }
    public double getErrorR() { return this.r - this.localization.getR(); }

    public boolean isBusy() {
        return this.busyEt < C.BUSY_ET_THRESHOLD;
    }

    public void update() {
        this.localization.update();
        this.timer.update();

        double errorX = this.getErrorX();
        double errorY = this.getErrorY();
        double errorR = this.getErrorR();
        double distance = Math.sqrt(errorX * errorX + errorY * errorY);
        double rotation = Math.abs(errorR);
        if (distance > C.DISTANCE_THRESHOLD || rotation > C.ROTATION_THRESHOLD) {
            this.busyEt -= this.timer.getDt();
        } else {
            this.busyEt += this.timer.getDt();
        }
        this.busyEt = M.clamp(this.busyEt, 0.0, C.BUSY_ET_THRESHOLD + C.EPSILON);

        this.xPid.update();
        this.yPid.update();
        this.rPid.update();
        this.drivetrain.update();
    }
}