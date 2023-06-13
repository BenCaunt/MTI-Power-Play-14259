package org.firstinspires.ftc.teamcode.pid;

import org.firstinspires.ftc.teamcode.input.Timer;
import org.firstinspires.ftc.teamcode.utils.M;

public class Pid {
    private ErrorFunction errorFunction;
    private ResponseFunction responseFunction;
    private Coefficients coefficients;
    private Timer timer;
    private double e = 0.0;
    private double et = 0.0;
    private double deDt = 0.0;

    public static class Coefficients {
        public double kp = 0.0, ki = 0.0, kd = 0.0, deDtGain = 0.0, etMax = 1e+99;

        public Coefficients(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
        }

        public Coefficients(double kp, double ki, double kd, double deDtGain, double etMax) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.deDtGain = deDtGain;
            this.etMax = etMax;
        }
    }

    public static interface ErrorFunction { public double execute(); }
    public static interface ResponseFunction { public void execute(double factor); }

    public Pid(Coefficients coefficients, ErrorFunction errorFunction, ResponseFunction responseFunction) {
        this.errorFunction = errorFunction;
        this.responseFunction = responseFunction;
        this.coefficients = coefficients;
        this.timer = new Timer();
    }

    public void update() {
        this.timer.update();

        double e = this.errorFunction.execute();
        double dt = this.timer.getDt();

        double deDt = M.lerp((e - this.e) / dt, this.deDt, this.coefficients.deDtGain);

        double det = e * dt;
        if (Math.signum(det) != Math.signum(this.et)) this.et = 0.0;
        double et = M.clamp(this.et + det, -this.coefficients.etMax, this.coefficients.etMax);

        double factor = 0.0;
        factor += this.coefficients.kp * e;
        factor += this.coefficients.ki * et;
        factor += this.coefficients.kd * deDt;

        this.e = e;
        this.et = et;
        this.deDt = deDt;

        this.responseFunction.execute(factor);
        //some cring epilepson stuff to save loop times cause min value or sm sm
    }
}