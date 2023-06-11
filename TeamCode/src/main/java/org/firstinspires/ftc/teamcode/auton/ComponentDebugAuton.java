package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;

@Autonomous
@Disabled
public class ComponentDebugAuton extends LinearOpMode {
    private G g;
    private AsyncThreaded donutAsync;

    public void initAsync() {
        AsyncThreaded head = new AsyncThreaded(() -> {});
        AsyncThreaded tail = head
                .then(() -> {
                    this.g.driveComponent.addTargetY(10);
                    this.g.driveComponent.addTargetR(Math.PI / 2.0);
                    while (this.g.driveComponent.isBusy() && !AsyncThreaded.stopped) this.g.driveComponent.update();
                })
                .then(() -> {
                    this.g.driveComponent.addTargetX(10);
                    this.g.driveComponent.addTargetR(Math.PI / 2.0);
                    while (this.g.driveComponent.isBusy() && !AsyncThreaded.stopped) this.g.driveComponent.update();
                })
                .then(() -> {
                    this.g.driveComponent.addTargetY(-10);
                    this.g.driveComponent.addTargetR(Math.PI / 2.0);
                    while (this.g.driveComponent.isBusy() && !AsyncThreaded.stopped) this.g.driveComponent.update();
                })
                .then(() -> {
                    this.g.driveComponent.addTargetX(-10);
                    this.g.driveComponent.addTargetR(Math.PI / 2.0);
                    while (this.g.driveComponent.isBusy() && !AsyncThreaded.stopped) this.g.driveComponent.update();
                });
        tail.then(head);
        this.donutAsync = head;
    }

    public void initAll() {
        this.g = new G(hardwareMap);
        this.g.initAll();
        this.initAsync();
    }

    public void updateAll() {
        g.updateAll();
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
