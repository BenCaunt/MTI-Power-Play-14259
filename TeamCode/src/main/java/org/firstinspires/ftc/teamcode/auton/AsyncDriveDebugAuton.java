package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.driveimpl.PosePidDrive;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;

@Autonomous
@Disabled
public class AsyncDriveDebugAuton extends LinearOpMode {
    private Drive drive;
    private AsyncThreaded donutAsync;

    private void initDrive() {
        this.drive = new PosePidDrive(hardwareMap);
    }

    private void initAsync() {
        AsyncThreaded head = new AsyncThreaded(() -> {});
        AsyncThreaded tail = head
            .then(() -> {
                telemetry.addData("Async", "1");
                this.drive.addTargetY(10);
                this.drive.addTargetR(Math.toRadians(90));
                while (this.drive.isBusy() && !AsyncThreaded.stopped) this.drive.update();
            })
            .then(() -> {
                telemetry.addData("Async", "2");
                this.drive.addTargetX(10);
                this.drive.addTargetR(Math.toRadians(90));
                while (this.drive.isBusy() && !AsyncThreaded.stopped) this.drive.update();
            })
            .then(() -> {
                telemetry.addData("Async", "3");
                this.drive.addTargetY(-10);
                this.drive.addTargetR(Math.toRadians(90));
                while (this.drive.isBusy() && !AsyncThreaded.stopped) this.drive.update();
            })
            .then(() -> {
                telemetry.addData("Async", "4");
                this.drive.addTargetX(-10);
                this.drive.addTargetR(Math.toRadians(90));
                while (this.drive.isBusy() && !AsyncThreaded.stopped) this.drive.update();
            });
        tail.then(head);
        this.donutAsync = head;
    }

    private void initAll() {
        this.initDrive();
        this.initAsync();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        this.donutAsync.run();
        while (opModeIsActive()) {

        }
        AsyncThreaded.stopped = true;
    }
}