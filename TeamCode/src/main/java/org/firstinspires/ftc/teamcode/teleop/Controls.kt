package org.firstinspires.ftc.teamcode.teleop

import org.firstinspires.ftc.teamcode.Constants.drive
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.driving.DriverControlled
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.*
import org.firstinspires.ftc.teamcode.util.CommandGamepad
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand

object Controls {
    private val gamepad1 = CommandGamepad(opMode.gamepad1)
    private val gamepad2 = CommandGamepad(opMode.gamepad2)

    fun registerGamepads() {
        CommandScheduler.registerGamepads(gamepad1, gamepad2)
    }

    fun registerCommands() {
        CommandScheduler.commands += drive.driverControlled(opMode.gamepad1)

        gamepad1.a.pressed.command = drive.switchSpeed
        gamepad2.a.pressed.command = Intake.switch
        gamepad2.b.pressed.command = Shooter.switch
    }
}