package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Constants.drive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.util.CommandGamepad
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.ParallelCommandGroup
import org.firstinspires.ftc.teamcode.util.commands.parallel

class CompTeleOp: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        drive.initialize()
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