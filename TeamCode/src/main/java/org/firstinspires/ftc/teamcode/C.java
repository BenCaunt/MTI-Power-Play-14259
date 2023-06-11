package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pid.Pid;

import java.util.concurrent.atomic.AtomicBoolean;

public class C {
    // General constants
    public static final double EPSILON = 0.01;

    // Pose PID drive constants
    public static final double DISTANCE_THRESHOLD = 1.1;
    public static final double ROTATION_THRESHOLD = 0.1;
    public static final Pid.Coefficients xPidCoefficients = new Pid.Coefficients(0.16 / C.POWER_X_SCALE, 0.005 / C.POWER_X_SCALE, 0.004 / C.POWER_X_SCALE, 0, 5 * C.POWER_X_SCALE);
    public static final Pid.Coefficients yPidCoefficients = C.xPidCoefficients;
    public static final Pid.Coefficients rPidCoefficients = new Pid.Coefficients(1, 0.09, 0.01, 1, 5);

    // Encoder constants
    public static final double INCHES_TO_TICKS = 1885.0;

    // Encoder Resets
    public static boolean encoderTuned = false;

    // Odometry constants
    public static final double PARA_SCALE = 1.0;
    public static final double PERP_SCALE = 1.0;

    // Two-wheel odometry constants
    public static final double PARA_OFFSET = 10.0;
    public static final double PERP_OFFSET = 10.0;

    // Three-wheel odometry constants
    public static final double TRACK_WIDTH = 12.87;
    public static final double LEFT_OFFSET = TRACK_WIDTH / 2;
    public static final double RIGHT_OFFSET = -TRACK_WIDTH / 2;
    public static final double CENTER_OFFSET = 6.3;

    // Drivetrain constants
    public static final double POWER_X_SCALE = 2.2;

    // Drive constants
    public static final double BUSY_ET_THRESHOLD = 0.2;

    // Camera constants
    public static final int CAMERA_WIDTH = 320;
    public static final int CAMERA_HEIGHT = 240;
    public static final double CAMERA_FOV = 72.0 / 180 * Math.PI;

    // Simulation constants
    public static final double SIMULATION_FRUSTUM = 0.1;

    //Threads
    public static AtomicBoolean parked = new AtomicBoolean(false);

    //Servo

    public static final double depositLB = 0.5;
    public static final double depositUB = 1;
    public static final double[] depositPositions= {0,0.2,1};
    public static final double latchLB = 0.38;
    public static final double latchUB = 0.65;
    public static final double[] latchPositions= {0,0.6};
    public static final double frontArmLB = 0.03;
    public static final double frontArmUB = 0.9;
    public static final double[] frontArmPositions = {0,1};
    public static final double clawLB = 0.45;
    public static final double clawUB = 0.85;
    public static final double[] clawPositions= {0.6,0};
    public static final double leftArmLB = 0.5;
    public static final double leftArmUB = .84;
    public static final double[] armPositions= {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
    public static final double rightArmLB = 0.5;
    public static final double rightArmUB = .84;
    public static final double[] rightArmPositions= {0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1};
    public static double getTargetFrontArmPosition(int i){
        if(i == 4) return 0.095;
        else if(i == 3)return 0.08;
        else if(i == 2)return 0.06;
        else if(i == 1)return 0.03;
        else if(i == 0) return 0.0;
        else if(i == 5) return 0.15;
        else return 0;
    }
    //Motor
    public static final double turretLB = 0;
    public static final double turretUB = -1455;
    public static final double[] turretPositions= {0,0.7,0.9};
    public static final double pitchLB = 0;
    public static final double pitchUB = -1875;
    public static final double[] pitchPositions= {0,0.5};
    public static final double linSlideLB = 0;
    public static final double linSlideUB = -1250;
    public static final double[] linSlidePositions= {0,1};
}