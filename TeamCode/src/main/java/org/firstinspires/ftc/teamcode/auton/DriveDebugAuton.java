package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Drive;
import org.firstinspires.ftc.teamcode.drive.driveimpl.PosePidDrive;

@Autonomous
@Disabled
public class DriveDebugAuton extends LinearOpMode {
    private Drive drive;
    private int state = 0;

    private void initDrive() {
        this.drive = new PosePidDrive(hardwareMap);
    }

    private void initAll() {
        this.initDrive();
    }

    private void updateDrive() {
        this.drive.update();
    }

    private void updateAll() {
        this.updateDrive();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();

        waitForStart();

        while (opModeIsActive()) {
            if (!this.drive.isBusy()) {
                switch (state) {
                    case 0:
                        state = 1;
                        this.drive.addTargetY(10);
                        this.drive.addTargetR(Math.PI / 2.0);
                        break;
                    case 1:
                        state = 2;
                        this.drive.addTargetX(10);
                        this.drive.addTargetR(Math.PI / 2.0);
                        break;
                    case 2:
                        state = 3;
                        this.drive.addTargetY(-10);
                        this.drive.addTargetR(Math.PI / 2.0);
                        break;
                    case 3:
                        state = 0;
                        this.drive.addTargetX(-10);
                        this.drive.addTargetR(Math.PI / 2.0);
                        break;
                }
            }

            this.updateAll();
        }
    }
}