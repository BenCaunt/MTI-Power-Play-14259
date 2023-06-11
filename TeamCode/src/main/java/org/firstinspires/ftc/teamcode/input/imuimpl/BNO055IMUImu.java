//package org.firstinspires.ftc.teamcode.input.imuimpl;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.teamcode.input.Imu;
//
//public class BNO055IMUImu implements Imu {
//    public static final AxesReference axesReference = AxesReference.INTRINSIC;
//    public static final AxesOrder axesOrder = AxesOrder.ZYX;
//    public static final AngleUnit angleUnit = AngleUnit.RADIANS;
//
//    private BNO055IMU imu;
//    private double ry = 0.0, rp = 0.0, rr = 0.0;
//    private double y = 0.0, p = 0.0, r = 0.0;
//    private double dy = 0.0, dp = 0.0, dr = 0.0;
//    private double dyDt = 0.0, dpDt = 0.0, drDt = 0.0;
//
//    public BNO055IMUImu(BNO055IMU imu) {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        imu.initialize(parameters);
//        while (!imu.isGyroCalibrated());
//
//        this.imu = imu;
//    }
//
//    public double getY() { return this.y; }
//    public double getP() { return this.p; }
//    public double getR() { return this.r; }
//    public double getDy() { return this.dy; }
//    public double getDp() { return this.dp; }
//    public double getDr() { return this.dr; }
//    public double getDyDt() { return this.dyDt; }
//    public double getDpDt() { return this.dpDt; }
//    public double getDrDt() { return this.drDt; }
//
//    public void update() {
//        Orientation ao = this.imu.getAngularOrientation(BNO055IMUImu.axesReference, BNO055IMUImu.axesOrder, BNO055IMUImu.angleUnit);
//        AngularVelocity av = this.imu.getAngularVelocity();
//
//        double ry = ao.firstAngle;
//        double rp = ao.secondAngle;
//        double rr = ao.thirdAngle;
//
//        double dy = ry - this.ry;
//        double dp = rp - this.rp;
//        double dr = rr - this.rr;
//
//        if (dy < -Math.PI) dy += Math.PI * 2.0;
//        if (dy > Math.PI) dy -= Math.PI * 2.0;
//        if (dp < -Math.PI) dp += Math.PI * 2.0;
//        if (dp > Math.PI) dp -= Math.PI * 2.0;
//        if (dr < -Math.PI) dr += Math.PI * 2.0;
//        if (dr > Math.PI) dr -= Math.PI * 2.0;
//
//        double y = this.y + dy;
//        double p = this.p + dp;
//        double r = this.r + dr;
//
//        double dyDt = av.zRotationRate;
//        double dpDt = av.yRotationRate;
//        double drDt = av.xRotationRate;
//
//        this.ry = ry;
//        this.rp = rp;
//        this.rr = rr;
//
//        this.y = y;
//        this.p = p;
//        this.r = r;
//
//        this.dy = dy;
//        this.dp = dp;
//        this.dr = dr;
//
//        this.dyDt = dyDt;
//        this.dpDt = dpDt;
//        this.drDt = drDt;
//    }
//}