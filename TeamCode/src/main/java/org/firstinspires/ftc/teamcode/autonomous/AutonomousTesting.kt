package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.util.commands.*
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay

@Autonomous(name = "Command Testing")
class AutonomousTesting : LinearOpMode() {
    val testingRoutine: AtomicCommand
        get() = sequential {
            +CustomCommand(_start = {
                MecanumDrive.telemetry.addLine("Started Outer Sequential")
                MecanumDrive.telemetry.update()
            })
            +Delay(5.0)
            +parallel {
                +CustomCommand(_start = {
                    MecanumDrive.telemetry.addLine("Started Parallel")
                    MecanumDrive.telemetry.update()
                })
                +Delay(5.0)
                +sequential {
                    +CustomCommand(_start = {
                        MecanumDrive.telemetry.addLine("Started Inner Sequential")
                        MecanumDrive.telemetry.update()
                    })
                    +Delay(5.0)
                    +CustomCommand(_start = {
                        MecanumDrive.telemetry.addLine("Finished Inner Sequential")
                        MecanumDrive.telemetry.update()
                    })
                }
                +Delay(5.0)
                +CustomCommand(_start = {
                    MecanumDrive.telemetry.addLine("Finished Parallel")
                    MecanumDrive.telemetry.update()
                })
            }
            +Delay(5.0)
            +CustomCommand(_start = {
                MecanumDrive.telemetry.addLine("Finished Outer Sequential")
                MecanumDrive.telemetry.update()
            })
        }

    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Intake.initialize()
        Shooter.initialize()
        Wobble.initialize()
        CommandScheduler.commands += sequential {
            +testingRoutine
        }

        waitForStart()

        while (opModeIsActive()) {
            CommandScheduler.run()
        }
    }
}