package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble

@Autonomous
class LocalizationTesting: LinearOpMode() {
    override fun runOpMode() {
        Constants.opMode = this

        MecanumDrive.initialize()
        Intake.initialize()
        Shooter.initialize()
        Wobble.initialize()

        waitForStart()
        while(opModeIsActive()) {
            MecanumDrive.periodic()
        }
    }
}