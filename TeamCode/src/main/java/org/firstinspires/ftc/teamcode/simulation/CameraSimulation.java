package org.firstinspires.ftc.teamcode.simulation;

import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.G;
import org.firstinspires.ftc.teamcode.utils.Pair;
import org.firstinspires.ftc.teamcode.utils.Vector2;
import org.firstinspires.ftc.teamcode.utils.Vector3;

import java.util.ArrayList;

public class CameraSimulation {
    public static final Junction[] junctions = {
        new LowJunction(24, 48),
        new LowJunction(24, 96),
        new LowJunction(48, 24),
        new LowJunction(48, 120),
        new LowJunction(96, 24),
        new LowJunction(96, 120),
        new LowJunction(120, 48),
        new LowJunction(120, 96),
        new MediumJunction(48, 48),
        new MediumJunction(48, 96),
        new MediumJunction(96, 48),
        new MediumJunction(96, 96),
        new HighJunction(48, 72),
        new HighJunction(72, 48),
        new HighJunction(72, 96),
        new HighJunction(96, 72)
    };

    public static class Point extends Vector3 {
        public Point() {
            super();
        }

        public Point(double x, double y, double z) {
            super(x, y, z);
        }

        public Point(Vector3 vector) {
            super(vector.x, vector.y, vector.z);
        }

        public boolean outOfBounds() {
            return this.z < C.SIMULATION_FRUSTUM;
        }

        public ScreenSpacePoint project() {
            double x = (1 - this.x / Math.abs(this.z) / C.CAMERA_FOV) * (double) C.CAMERA_WIDTH / 2;
            double y = (1 - this.y / Math.abs(this.z) / C.CAMERA_FOV) * (double) C.CAMERA_HEIGHT / 2;
            return new ScreenSpacePoint(x, y);
        }
    }

    public static class Line {
        public Point a, b;

        public Line(Point a, Point b) {
            this.a = a;
            this.b = b;
        }

        public Line rotateYaw(double theta) {
            return new Line(new Point(this.a.rotateYaw(theta)), new Point(this.b.rotateYaw(theta)));
        }

        public Line rotatePitch(double theta) {
            return new Line(new Point(this.a.rotatePitch(theta)), new Point(this.b.rotatePitch(theta)));
        }

        public Line rotateRoll(double theta) {
            return new Line(new Point(this.a.rotateRoll(theta)), new Point(this.b.rotateRoll(theta)));
        }

        public Line rotate(Vector3 rotation) {
            return new Line(new Point(this.a.rotate(rotation)), new Point(this.b.rotate(rotation)));
        }

        public Line translate(Vector3 translation) {
            return new Line(new Point(this.a.translate(translation)), new Point(this.b.translate(translation)));
        }

        public Line cull() {
            Point a = this.a, b = this.b;
            if (a.outOfBounds() && b.outOfBounds()) return null;
            double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
            if (a.outOfBounds()) {
                double w = (C.SIMULATION_FRUSTUM - b.z) / dz;
                a = new Point(b.x + dx * w, b.y + dy * w, C.SIMULATION_FRUSTUM);
            }
            if (b.outOfBounds()) {
                double w = (C.SIMULATION_FRUSTUM - a.z) / dz;
                b = new Point(a.x + dx * w, a.y + dy * w, C.SIMULATION_FRUSTUM);
            }
            return new Line(a, b);
        }

        public ScreenSpaceLine project() {
            return new ScreenSpaceLine(this.a.project(), this.b.project());
        }
    }

    public static class ScreenSpacePoint extends Vector2 {
        public ScreenSpacePoint(double x, double y) {
            super(x, y);
        }

        public ScreenSpacePoint(Vector2 vector) {
            super(vector.x, vector.y);
        }

        public boolean outOfBounds() {
            return this.x < 0 || this.x >= C.CAMERA_WIDTH || this.y < 0 || this.y >= C.CAMERA_HEIGHT;
        }
    }

    public static class ScreenSpaceLine {
        public ScreenSpacePoint a, b;

        public ScreenSpaceLine(ScreenSpacePoint a, ScreenSpacePoint b) {
            this.a = a;
            this.b = b;
        }

        public ScreenSpaceLine cull() {
            ScreenSpacePoint a = this.a, b = this.b;
            double dx = b.x - a.x, dy = b.y - a.y;
            double wA = 1, wB = 1;
            if (dx > 0) {
                wB = Math.min(wB, (C.CAMERA_WIDTH - a.x) / dx);
                wA = Math.min(wA, (b.x - 0) / dx);
            }
            if (dx < 0) {
                wA = Math.min(wA, (b.x - C.CAMERA_WIDTH) / dx);
                wB = Math.min(wB, (0 - a.x) / dx);
            }
            if (dy > 0) {
                wB = Math.min(wB, (C.CAMERA_HEIGHT - a.y) / dy);
                wA = Math.min(wA, (b.y - 0) / dy);
            }
            if (dy < 0) {
                wA = Math.min(wA, (b.y - C.CAMERA_HEIGHT) / dy);
                wB = Math.min(wB, (0 - a.y) / dy);
            }
            ScreenSpacePoint culledA = new ScreenSpacePoint(b.x - dx * wA, b.y - dy * wA);
            ScreenSpacePoint culledB = new ScreenSpacePoint(a.x + dx * wB, a.y + dy * wB);
            if (culledA.outOfBounds() || culledB.outOfBounds()) return null;
            return new ScreenSpaceLine(culledA, culledB);
        }
    }

    public static class Junction {
        private double x, z;
        private double height;
        private Line line;

        public Junction(double x, double z, double height) {
            this.x = x;
            this.z = z;
            this.height = height;
            this.line = new Line(new Point(x, 0, z), new Point(x, height, z));
        }

        public ScreenSpaceLine render() {
            Vector3 transformPosition = G.context.cameraComponent.transformPosition(new Vector3());
            Vector3 transformRotation = G.context.cameraComponent.transformRotation(new Vector3());
            Line renderReady = this.line
                    .translate(transformPosition.negate())
                    .rotate(transformRotation.negate())
                    .cull();
            if (renderReady == null) return null;
            return renderReady
                    .project()
                    .cull();
        }
    }

    public static class LowJunction extends Junction {
        public LowJunction(double x, double z) {
            super(x, z, 10);
        }
    }

    public static class MediumJunction extends Junction {
        public MediumJunction(double x, double z) {
            super(x, z, 20);
        }
    }

    public static class HighJunction extends Junction {
        public HighJunction(double x, double z) {
            super(x, z, 30);
        }
    }

    public static ArrayList<Pair<ScreenSpaceLine, Junction>> render() {
        ArrayList<Pair<ScreenSpaceLine, Junction>> dataPairs = new ArrayList<>();
        for (Junction junction : junctions) {
            ScreenSpaceLine screenSpaceLine = junction.render();
            if (screenSpaceLine != null) dataPairs.add(new Pair(screenSpaceLine, junction));
        }
        return dataPairs;
    }
}
