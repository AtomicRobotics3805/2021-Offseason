package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive

object Constants {
    enum class Color {
        BLUE,
        RED
    }

    var startPose = Pose2d()
    var color = Color.BLUE
    val drive = MecanumDrive
    lateinit var opMode: OpMode
}