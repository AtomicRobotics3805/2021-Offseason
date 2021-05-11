package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("unused")
class CompTeleOp: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Intake.initialize()
        Shooter.initialize()
        Wobble.initialize()
        Controls.registerGamepads()

        waitForStart()

        Controls.registerCommands()

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}