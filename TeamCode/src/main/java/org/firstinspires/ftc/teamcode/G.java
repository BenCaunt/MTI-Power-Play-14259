package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.components.CameraComponent;
import org.firstinspires.ftc.teamcode.components.Component;
import org.firstinspires.ftc.teamcode.components.DriveComponent;
import org.firstinspires.ftc.teamcode.components.LinearSlideComponent;
import org.firstinspires.ftc.teamcode.components.PitchComponent;
import org.firstinspires.ftc.teamcode.components.TurretComponent;

public class G {
    public static G context;

    public HardwareMap hardwareMap;
    public DriveComponent driveComponent = null;
    public TurretComponent turretComponent = null;
    public PitchComponent pitchComponent = null;
    public LinearSlideComponent linearSlideComponent = null;
    public CameraComponent cameraComponent = null;

    public G(HardwareMap hardwareMap) {
        G.context = this;
        this.hardwareMap = hardwareMap;
    }

    public void initDriveComponent() {
        if (this.driveComponent != null) return;
        this.driveComponent = new DriveComponent(this.hardwareMap);
    }

    public void initTurretComponent() {
        if (this.turretComponent != null) return;
        this.initDriveComponent();
        this.turretComponent = new TurretComponent(this.driveComponent, this.hardwareMap);
    }

    public void initPitchComponent() {
        if (this.pitchComponent != null) return;
        this.initTurretComponent();
        this.pitchComponent = new PitchComponent(this.turretComponent, this.hardwareMap);
    }

    public void initLinearSlideComponent() {
        if (this.linearSlideComponent != null) return;
        this.initPitchComponent();
        this.linearSlideComponent = new LinearSlideComponent(this.pitchComponent, this.hardwareMap);
    }

    public void initCameraComponent() {
        if (this.cameraComponent != null) return;
        this.initPitchComponent();
        this.cameraComponent = new CameraComponent(this.pitchComponent, this.hardwareMap);
    }

    public void initAll() {
        this.initDriveComponent();
        this.initTurretComponent();
        this.initPitchComponent();
        this.initLinearSlideComponent();
        this.initCameraComponent();
    }

    public void updateDriveComponent() {
        if (this.driveComponent == null) return;
        this.driveComponent.update();
    }

    public void updateTurretComponent() {
        if (this.turretComponent == null) return;
        this.turretComponent.update();
    }

    public void updatePitchComponent() {
        if (this.pitchComponent == null) return;
        this.pitchComponent.update();
    }

    public void updateLinearSlideComponent() {
        if (this.linearSlideComponent == null) return;
        this.linearSlideComponent.update();
    }

    public void updateAll() {
        this.updateDriveComponent();
        this.updateTurretComponent();
        this.updatePitchComponent();
        this.updateLinearSlideComponent();
    }
}
