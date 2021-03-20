package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp

object Singleton {
    enum class Color {
        BLUE,
        RED
    }

    var startPose = Pose2d()
    val color = Color.BLUE
    val constants: BaseDriveConstants = DriveConstantsComp
    lateinit var drive: BaseMecanumDrive
    lateinit var opMode: OpMode
}