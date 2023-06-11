package org.firstinspires.ftc.teamcode.utils;

public class Vector3 {
    public double x, y, z;

    public Vector3() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3 negate() {
        return new Vector3(-this.x, -this.y, -this.z);
    }

    public Vector3 add(Vector3 that) {
        return new Vector3(this.x + that.x, this.y + that.y, this.z + that.z);
    }

    public Vector3 add(double that) {
        return this.add(new Vector3(that, that, that));
    }

    public Vector3 subtract(Vector3 that) {
        return new Vector3(this.x - that.x, this.y - that.y, this.z - that.z);
    }

    public Vector3 subtract(double that) {
        return this.subtract(new Vector3(that, that, that));
    }

    public Vector3 multiply(Vector3 that) {
        return new Vector3(this.x * that.x, this.y * that.y, this.z * that.z);
    }

    public Vector3 multiply(double that) {
        return this.multiply(new Vector3(that, that, that));
    }

    public Vector3 divide(Vector3 that) {
        return new Vector3(this.x / that.x, this.y / that.y, this.z / that.z);
    }

    public Vector3 divide(double that) {
        return this.divide(new Vector3(that, that, that));
    }

    public double dot(Vector3 that) {
        return this.x * that.x + this.y * that.y + this.z * that.z;
    }

    public double length() {
        return Math.sqrt(this.dot(this));
    }

    public Vector3 normalize() {
        return this.divide(this.length());
    }

    public Vector3 rotateYaw(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Vector3(this.z * sin + this.x * cos, this.y, this.z * cos - this.x * sin);
    }

    public Vector3 rotatePitch(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Vector3(this.x, this.z * sin + this.y * cos, this.z * cos - this.y * sin);
    }

    public Vector3 rotateRoll(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Vector3(this.x * cos - this.y * sin, this.x * sin + this.y * cos, this.z);
    }

    public Vector3 rotate(Vector3 rotation) {
        return this.rotateYaw(rotation.x).rotatePitch(rotation.y).rotateRoll(rotation.z);
    }

    public Vector3 translate(Vector3 translation) {
        return this.add(translation);
    }

    public String toString() {
        return "(" + this.x + ", " + this.y + ", " + this.z + ")";
    }
}
