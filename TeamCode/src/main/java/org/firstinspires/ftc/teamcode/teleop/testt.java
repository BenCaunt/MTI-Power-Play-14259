package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.output.goBildaTouchDriver;
@Disabled
@TeleOp
public class testt extends LinearOpMode {
    DcMotorEx testMotor;
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        goBildaTouchDriver Touch = new goBildaTouchDriver(hardwareMap.get(DigitalChannel.class, "pitches"));
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            boolean e = Touch.check();
            if(!e) testMotor.setPower(0.1);
            else testMotor.setPower(0);
        telemetry.addData("state", Touch.check());
        telemetry.update();

        }
    }
}
