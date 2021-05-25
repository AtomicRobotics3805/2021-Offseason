package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Autonomous(name = "Command Testing")
class AutonomousTesting : LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Intake.initialize()
        Shooter.initialize()
        Wobble.initialize()
        CommandScheduler.commands += sequential {
            +AutoRoutines.initRoutine
            +AutoRoutines.lowRoutine
        }

        waitForStart()

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}