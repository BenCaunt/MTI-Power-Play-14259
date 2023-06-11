package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.DashboardUtil;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
public class BackAndForth extends LinearOpMode {
    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);
    private PIDFController translationalController = new PIDFController(SampleMecanumDrive.TRANSLATIONAL_PID);

//    public static double DISTANCE = 20;
    private Vector2d targetPosition = new Vector2d(0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.getLocalizer().setPoseEstimate(new Pose2d(0,0,Math.toRadians(180)));
        headingController.setInputBounds(-Math.PI, Math.PI);
//        translationalController.setInputBounds(-DISTANCE, DISTANCE);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();
            targetPosition = new Vector2d(poseEstimate.getX(), poseEstimate.getY());
            Vector2d difference = targetPosition.minus(poseEstimate.vec());
            // Obtain the target angle for feedback and derivative for feedforward
            double theta = difference.angle();

            // Not technically omega because its power. This is the derivative of atan2

            // Set the target heading for the heading controller to our desired angle
            headingController.setTargetPosition(theta);
//            headingController.setTargetPosition(Math.toRadians(0));
//            translationalController.setTargetPosition(DISTANCE);
            double headinginput = headingController.update(poseEstimate.getHeading());
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y,-gamepad1.left_stick_x,headinginput));

            drive.getLocalizer().update();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("theta", theta);
            telemetry.update();
        }
    }
}