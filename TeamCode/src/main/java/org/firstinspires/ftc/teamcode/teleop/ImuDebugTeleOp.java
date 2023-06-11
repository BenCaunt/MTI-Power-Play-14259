//package org.firstinspires.ftc.teamcode.teleop;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.drivetrain.drivetrainimpl.MecanumDrivetrain;
//import org.firstinspires.ftc.teamcode.input.Imu;
//import org.firstinspires.ftc.teamcode.input.imuimpl.BNO055IMUImu;
//
//@TeleOp
//@Disabled
//public class ImuDebugTeleOp extends LinearOpMode {
//    private Drivetrain drivetrain;
//    private Imu imu;
//
//    private void initDrivetrain() {
//        this.drivetrain = new MecanumDrivetrain(hardwareMap);
//    }
//
//    private void initImu() { this.imu = new BNO055IMUImu(hardwareMap.get(BNO055IMU.class, "imu")); }
//
//    private void initAll() {
//        this.initDrivetrain();
//        this.initImu();
//    }
//
//    private void updateDrivetrain() { this.drivetrain.update(); }
//
//    private void updateImu() { this.imu.update(); }
//
//    private void updateTelemetry() {
//        telemetry.addData("this.imu.getY()", this.imu.getY());
//        telemetry.addData("this.imu.getDy()", this.imu.getDy());
//        telemetry.addData("this.imu.getDyDt()", this.imu.getDyDt());
//        telemetry.update();
//    }
//
//    private void updateAll() {
//        this.updateDrivetrain();
//        this.updateImu();
//        this.updateTelemetry();
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        this.initAll();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            double r = this.imu.getY();
//            this.drivetrain.addPower(gamepad1.left_stick_x, r);
//            this.drivetrain.addPower(gamepad1.left_stick_y, r + Math.PI / 2.0);
//            this.drivetrain.addPowerR(gamepad1.right_stick_x);
//
//            this.updateAll();
//        }
//    }
//}