package org.firstinspires.ftc.teamcode.input.imuimpl;//package org.firstinspires.ftc.teamcode.input.imuimpl;
//
//import com.qualcomm.robotcore.hardware.IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
//
//import org.firstinspires.ftc.teamcode.input.Imu;
//
//public class IMUImu implements Imu {
//    public static final AngleUnit angleUnit = AngleUnit.RADIANS;
//
//    private IMU imu;
//    private double ry = 0.0, rp = 0.0, rr = 0.0;
//    private double y = 0.0, p = 0.0, r = 0.0;
//    private double dy = 0.0, dp = 0.0, dr = 0.0;
//    private double dyDt = 0.0, dpDt = 0.0, drDt = 0.0;
//
//    public IMUImu(IMU imu) { this.imu = imu; }
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
//        YawPitchRollAngles ao = this.imu.getRobotYawPitchRollAngles();
//        AngularVelocity av = this.imu.getRobotAngularVelocity(IMUImu.angleUnit);
//
//        double ry = ao.getYaw(IMUImu.angleUnit);
//        double rp = ao.getPitch(IMUImu.angleUnit);
//        double rr = ao.getRoll(IMUImu.angleUnit);
//
//        double dy = ry - this.ry;
//        double dp = rp - this.rp;
//        double dr = rr - this.rr;
//
//        if (dy < -Math.PI) dy += Math.PI * 2.0;
//        else dy -= Math.PI * 2.0;
//        if (dp < -Math.PI) dp += Math.PI * 2.0;
//        else dr -= Math.PI * 2.0;
//        if (dr < -Math.PI) dp += Math.PI * 2.0;
//        else dr -= Math.PI * 2.0;
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