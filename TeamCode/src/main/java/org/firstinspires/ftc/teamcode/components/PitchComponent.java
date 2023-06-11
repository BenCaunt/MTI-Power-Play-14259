package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorMotor;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.M;

@Config
public class PitchComponent extends DcMotorMotor implements Component {
    private static final double RADIANS_TO_TICKS = -2289;
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = -1875;
    private static final double INIT_RADIANS = 0.12217304763960307;

    private Component parent;

    public PitchComponent(Component parent, HardwareMap hardwareMap) {
        super(hardwareMap.get(DcMotor.class, "pitch"));
        this.parent = parent;
        this.setLowerBound(PitchComponent.LOWER_BOUND);
        this.setUpperBound(PitchComponent.UPPER_BOUND);
    }

    public PitchComponent setRadians(double radians) {
        this.setPosition(M.normalize((radians - PitchComponent.INIT_RADIANS) * PitchComponent.RADIANS_TO_TICKS, PitchComponent.LOWER_BOUND, PitchComponent.UPPER_BOUND));
        return this;
    }

    public double getCurrentRadians() {
        return M.lerp(PitchComponent.LOWER_BOUND, PitchComponent.UPPER_BOUND, this.getCurrentPosition()) / PitchComponent.RADIANS_TO_TICKS + PitchComponent.INIT_RADIANS;
    }

    public double getTargetRadians() {
        return M.lerp(PitchComponent.LOWER_BOUND, PitchComponent.UPPER_BOUND, this.getTargetPosition()) / PitchComponent.RADIANS_TO_TICKS + PitchComponent.INIT_RADIANS;
    }

    public Component getParent() {
        return this.parent;
    }

    public Vector3 transformPosition(Vector3 position) {
        return this.parent.transformPosition(position.rotatePitch(this.getCurrentRadians()));
    }

    public Vector3 transformRotation(Vector3 rotation) {
        return this.parent.transformRotation(rotation.add(new Vector3(0, this.getCurrentRadians(), 0)));
    }
}
