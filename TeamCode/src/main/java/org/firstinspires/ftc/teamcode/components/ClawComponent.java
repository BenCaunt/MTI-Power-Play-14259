package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.utils.Vector3;

public class ClawComponent extends ServoMotor implements Component {
    private static final double LOWER_BOUND = 0.5;
    private static final double UPPER_BOUND = 1;

    private Component parent;
    private ServoMotor motor;

    public ClawComponent(Component parent, HardwareMap hardwareMap) {
        super(hardwareMap.get(Servo.class, "claw"));
        this.parent = parent;
        this.setLowerBound(ClawComponent.LOWER_BOUND);
        this.setUpperBound(ClawComponent.UPPER_BOUND);
    }

    public Component getParent() {
        return this.parent;
    }

    public Vector3 transformPosition(Vector3 position) {
        return this.parent.transformPosition(position);
    }

    public Vector3 transformRotation(Vector3 position) {
        return this.parent.transformRotation(position);
    }
}
