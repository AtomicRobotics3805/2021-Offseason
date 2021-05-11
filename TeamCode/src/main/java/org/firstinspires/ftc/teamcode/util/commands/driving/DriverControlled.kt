package org.firstinspires.ftc.teamcode.util.commands.driving

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

class DriverControlled(private val gamepad: Gamepad,
                       private val reverseStrafe: Boolean = true,
                       private val reverseStraight: Boolean = true,
                       private val reverseTurn: Boolean = true) : AtomicCommand() {
    override val _isDone = false

    override fun execute() {
        val drivePower = Pose2d(
                if (reverseStraight) {-1} else {1} * (gamepad.left_stick_y).toDouble(),
                if (reverseStrafe) {-1} else {1} * (gamepad.left_stick_x).toDouble(),
                if (reverseTurn) {-1} else {1} * (gamepad.right_stick_x).toDouble()
        )

        MecanumDrive.setWeightedDrivePower(drivePower * MecanumDrive.driverSpeed)
    }
}