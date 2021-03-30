package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.util.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.util.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble

object Singleton {
    enum class Color {
        BLUE,
        RED
    }

    var startPose = Pose2d()
    var color = Color.BLUE
    val constants: BaseDriveConstants = DriveConstantsComp
    lateinit var opMode: OpMode
    lateinit var drive: BaseMecanumDrive
    lateinit var intake: Intake
    lateinit var shooter: Shooter
    lateinit var wobble: Wobble
}