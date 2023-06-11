package org.firstinspires.ftc.teamcode.utils;

public class Vector2 {
    public double x, y;

    public Vector2() {
        this.x = 0;
        this.y = 0;
    }

    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2 negate() {
        return new Vector2(-this.x, -this.y);
    }

    public Vector2 add(Vector2 that) {
        return new Vector2(this.x + that.x, this.y + that.y);
    }

    public Vector2 add(double that) {
        return this.add(new Vector2(that, that));
    }

    public Vector2 subtract(Vector2 that) {
        return new Vector2(this.x - that.x, this.y - that.y);
    }

    public Vector2 subtract(double that) {
        return this.subtract(new Vector2(that, that));
    }

    public Vector2 multiply(Vector2 that) {
        return new Vector2(this.x * that.x, this.y * that.y);
    }

    public Vector2 multiply(double that) {
        return this.multiply(new Vector2(that, that));
    }

    public Vector2 divide(Vector2 that) {
        return new Vector2(this.x / that.x, this.y / that.y);
    }

    public Vector2 divide(double that) {
        return this.divide(new Vector2(that, that));
    }

    public double dot(Vector2 that) {
        return this.x * that.x + this.y * that.y;
    }

    public double length() {
        return Math.sqrt(this.dot(this));
    }

    public Vector2 normalize() {
        return this.divide(this.length());
    }

    public Vector2 rotate(double theta) {
        double cos = Math.cos(theta), sin = Math.sin(theta);
        return new Vector2(this.x * cos - this.y * sin, this.x * sin + this.y * cos);
    }

    public Vector2 translate(Vector2 translation) {
        return this.add(translation);
    }

    public String toString() {
        return "(" + this.x + ", " + this.y + ")";
    }
}
