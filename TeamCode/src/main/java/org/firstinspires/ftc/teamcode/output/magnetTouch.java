package org.firstinspires.ftc.teamcode.output;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
this is supposed to go under "input"
*/

public class magnetTouch {
    private TouchSensor s;

    public magnetTouch(TouchSensor s) {
        this.s = s;
    }

    public boolean check() {
        boolean e = this.s.isPressed();
        return e;
    }

}
