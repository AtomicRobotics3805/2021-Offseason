package org.firstinspires.ftc.teamcode.tuning.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.SampleMecanumDrive;

@Config
@Autonomous(group = "drive")
public class ComplicatedStrafeTesting extends LinearOpMode {
    public static Vector2d Vector_1 = new Vector2d(30, -30);
    public static double heading = Math.toRadians(0);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(Vector_1)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(trajectory);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        drive.followTrajectory(
                drive.trajectoryBuilder(trajectory.end(), true)
                .strafeTo(new Vector2d(0, 0))
                .build());

        while (!isStopRequested() && opModeIsActive()) ;
    }
}
