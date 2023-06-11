package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.drive.driveimpl.PosePidDrive;
import org.firstinspires.ftc.teamcode.utils.Vector3;

/*
TODO: Generalize initialization to function
 */

public class DriveComponent extends PosePidDrive implements Component {
    public static final double INIT_X = 136;
    public static final double INIT_Y = 108;
    public static final double INIT_R = 0;
    public static final Vector3 POSITION_OFFSET = new Vector3(0, 5.375, 0);

    public DriveComponent(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public Component getParent() {
        return null;
    }

    public Vector3 transformPosition(Vector3 position) {
        return position.rotateYaw(this.getCurrentR()).add(DriveComponent.POSITION_OFFSET.add(new Vector3(this.getCurrentX(), 0, this.getCurrentY())));
    }

    public Vector3 transformRotation(Vector3 rotation) {
        return rotation.add(new Vector3(this.getCurrentR(), 0, 0));
    }
}
