package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.utils.Vector3;

public interface Component {
    public Component getParent();
    public Vector3 transformPosition(Vector3 position);
    public Vector3 transformRotation(Vector3 position);
}
