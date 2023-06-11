//package org.firstinspires.ftc.teamcode.auton;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.drive.Drive;
//import org.firstinspires.ftc.teamcode.drive.driveimpl.PosePidDrive;
//import org.firstinspires.ftc.teamcode.input.AsyncThreaded;
//@Disabled
//@Autonomous
//public class AsyncShitheadAuton extends LinearOpMode {
//    private Drive drive;
//    private AsyncThreaded donutAsync;
//
//    private void initDrive() {
//        this.drive = new PosePidDrive(hardwareMap);
//    }
//
//    private void initAsync() {
//        AsyncThreaded mainThread = new AsyncThreaded(() -> {})
//                .then(() -> {
//                    this.drive.addTargetY(10);
//                    while (this.drive.isBusy() && !AsyncThreaded.stopped.get()) this.drive.update();
//                })
//                .then(() -> {
//                    this.drive.addTargetX(10);
//                    while (this.drive.isBusy() && !AsyncThreaded.stopped.get()) this.drive.update();
//                })
//                .then(() -> {
//                    this.drive.addTargetY(-10);
//                    while (this.drive.isBusy() && !AsyncThreaded.stopped.get()) this.drive.update();
//                })
//                .then(() -> {
//                    this.drive.addTargetX(-10);
//                    while (this.drive.isBusy() && !AsyncThreaded.stopped.get()) this.drive.update();
//                });
//        AsyncThreaded radnomT = new AsyncThreaded(() -> {
//            while (!AsyncThreaded.stopped.get()) this.drive.update();
//        });
//        //AsyncThreaded cycleThread = new AsyncThreaded(cycleStuff);
//        //AsyncThreaded lastThread = ...
//        mainThread.then(radnomT);
//        //mainThread.then(cycleThread);
//        //radnomT.then(lastThread);
//        //cycleThread.then(lastThread);
//    }
//
//    private void initAll() {
//        this.initDrive();
//        this.initAsync();
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        this.initAll();
//
//        waitForStart();
//
//        this.donutAsync.run();
//        while (opModeIsActive()) {
//
//        }
//        AsyncThreaded.stopped.set(true);
//    }
//}