//package org.firstinspires.ftc.teamcode.localization.localizationimpl;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.C;
//import org.firstinspires.ftc.teamcode.input.Encoder;
//import org.firstinspires.ftc.teamcode.input.Imu;
//import org.firstinspires.ftc.teamcode.input.encoderimpl.DcMotorExEncoder;
//import org.firstinspires.ftc.teamcode.input.imuimpl.BNO055IMUImu;
//import org.firstinspires.ftc.teamcode.localization.Localization;
//
//public class TwoWheelLocalization implements Localization {
//    private Encoder paraEncoder;
//    private Encoder perpEncoder;
//    private Imu imu;
//    private double x = 0.0, y = 0.0, r = 0.0;
//
//    public TwoWheelLocalization(HardwareMap hardwareMap) {
//        this.paraEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "ParaEncoder"));
//        this.perpEncoder = new DcMotorExEncoder(hardwareMap.get(DcMotorEx.class, "PerpEncoder"));
//        this.imu = new BNO055IMUImu(hardwareMap.get(BNO055IMU.class, "imu"));
//    }
//
//    public double getX() { return this.x; }
//    public double getY() { return this.y; }
//    public double getR() { return this.r; }
//
//    public void update() {
//        this.paraEncoder.update();
//        this.perpEncoder.update();
//        this.imu.update();
//
//        double paraDelta = this.paraEncoder.getDx() * C.PARA_SCALE - C.PARA_OFFSET * this.imu.getDy();
//        double perpDelta = this.perpEncoder.getDx() * C.PERP_SCALE + C.PERP_OFFSET * this.imu.getDy();
//        double xDelta = perpDelta * Math.sin(this.r) - paraDelta * Math.cos(this.r);
//        double yDelta = perpDelta * Math.cos(this.r) + paraDelta * Math.sin(this.r);
//
//        this.x += xDelta;
//        this.y += yDelta;
//        this.r = this.imu.getY();
//    }
//}