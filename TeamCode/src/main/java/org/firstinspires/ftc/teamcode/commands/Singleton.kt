package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp

object Singleton {
    val constants: BaseDriveConstants = DriveConstantsComp
    lateinit var drive: BaseMecanumDrive
}