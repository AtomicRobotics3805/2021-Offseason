package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.util.trajectoryBuilder

@Autonomous
class ExampleAuto: LinearOpMode() {
    private val startPose = Pose2d()

    override fun runOpMode() {
        Singleton.drive = MecanumDriveComp(hardwareMap, Singleton.constants)
        sequential {
            +FollowTrajectory(trajectoryBuilder(startPose).build())
            +Turn(45.0.toRadians)
            +FollowTrajectory(trajectoryBuilder(startPose).build())
        }
    }
}