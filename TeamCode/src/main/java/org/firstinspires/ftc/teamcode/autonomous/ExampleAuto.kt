package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.commands.FollowTrajectory
import org.firstinspires.ftc.teamcode.Singleton
import org.firstinspires.ftc.teamcode.commands.Turn
import org.firstinspires.ftc.teamcode.commands.sequential
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.util.trajectories.trajectoryBuilder
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