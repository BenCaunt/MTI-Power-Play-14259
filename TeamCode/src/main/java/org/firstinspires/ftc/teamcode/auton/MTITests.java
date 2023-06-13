package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class MTITests extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double a = 1;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(-90)));
        waitForStart();
        if (isStopRequested()) return;
        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-36, -60, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-36, -2, Math.toRadians(-90)))
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(traj.end())
                .lineToLinearHeading(new Pose2d(-36, -12, Math.toRadians(-180)))
                .build();
        while(opModeIsActive()) {
            if(a == 1) {
                while(a == 1) {
                    drive.followTrajectory(traj);
                    drive.followTrajectory(traj1);
                    a=2;
                }
            }

            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
        }

    }
}


