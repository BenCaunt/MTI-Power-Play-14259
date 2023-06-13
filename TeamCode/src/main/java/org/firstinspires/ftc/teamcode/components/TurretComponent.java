package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.output.motorimpl.DcMotorMotor;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.firstinspires.ftc.teamcode.utils.M;

public class TurretComponent extends DcMotorMotor implements Component {
    private static final double RADIANS_TO_TICKS = -300; //103.8 * 28 = 2906.4 per 360 degrees
    private static final double LOWER_BOUND = 0;
    private static final double UPPER_BOUND = -1285;
    private static final double INIT_RADIANS = PI/2;

    private Component parent;

    public TurretComponent(Component parent, HardwareMap hardwareMap) {
        super(hardwareMap.get(DcMotor.class, "turret"));
        this.parent = parent;
        this.setLowerBound(TurretComponent.LOWER_BOUND);
        this.setUpperBound(TurretComponent.UPPER_BOUND);
    }

    public TurretComponent setRadians(double radians) {
        this.setPosition(M.normalize((radians - TurretComponent.INIT_RADIANS) * TurretComponent.RADIANS_TO_TICKS, TurretComponent.LOWER_BOUND, TurretComponent.UPPER_BOUND));
        return this;
    }

    public double getCurrentRadians() {
        return M.lerp(TurretComponent.LOWER_BOUND, TurretComponent.UPPER_BOUND, this.getCurrentPosition()) / TurretComponent.RADIANS_TO_TICKS + TurretComponent.INIT_RADIANS;
    }

    public double getTargetRadians() {
        return M.lerp(TurretComponent.LOWER_BOUND, TurretComponent.UPPER_BOUND, this.getTargetPosition()) / TurretComponent.RADIANS_TO_TICKS + TurretComponent.INIT_RADIANS;
    }

    public Component getParent() {
        return this.parent;
    }

    public Vector3 transformPosition(Vector3 position) {
        return this.parent.transformPosition(position.rotateYaw(this.getCurrentRadians()));
    }

    public Vector3 transformRotation(Vector3 rotation) {
        return this.parent.transformRotation(rotation.add(new Vector3(this.getCurrentRadians(), 0, 0)));
    }
}
