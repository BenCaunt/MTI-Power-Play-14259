package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.G;

@TeleOp
@Disabled
public class ComponentDebugTeleOp extends LinearOpMode {
    private G g;

    @Override
    public void runOpMode() throws InterruptedException {
        this.g = new G(hardwareMap);
        this.g.initAll();

        waitForStart();

        while (opModeIsActive()) {
            this.g.turretComponent.setPower(this.gamepad1.left_stick_y * 0.3);
            this.g.pitchComponent.setPower(this.gamepad1.right_stick_y * 1.0);

            telemetry.addData("this.g.turretComponent.getCurrentRadians()", this.g.turretComponent.getCurrentRadians());
            telemetry.addData("this.g.pitchComponent.getCurrentRadians()", this.g.pitchComponent.getCurrentRadians());
            telemetry.update();

            this.g.updateAll();
        }
    }
}
