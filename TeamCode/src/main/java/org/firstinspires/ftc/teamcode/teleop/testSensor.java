package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Disabled
@TeleOp
public class testSensor extends LinearOpMode {
    TouchSensor limit;


    @Override
    public void runOpMode() {
        limit = hardwareMap.get(TouchSensor.class, "Limit");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("limit", limit.isPressed());
            telemetry.update();
        }

    }
}