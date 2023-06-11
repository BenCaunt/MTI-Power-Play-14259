package org.firstinspires.ftc.teamcode.output;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
this is supposed to go under "input"
*/

public class goBildaTouchDriver {
    private DigitalChannel s;

    public goBildaTouchDriver(DigitalChannel s) {
        this.s = s;
    }

    public boolean check() {
        boolean e = this.s.getState();
        return e;
    }

}
