package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometryLocalizer
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
            MecanumDrive.telemetry.addData("Left Wheel", (MecanumDrive.localizer as OdometryLocalizer).leftEncoder.currentPosition)
            MecanumDrive.telemetry.addData("Right Wheel", (MecanumDrive.localizer as OdometryLocalizer).rightEncoder.currentPosition)
            MecanumDrive.telemetry.addData("Back Wheel", (MecanumDrive.localizer as OdometryLocalizer).frontEncoder.currentPosition)
            MecanumDrive.telemetry.addData("Position", MecanumDrive.localizer.poseEstimate)
            MecanumDrive.telemetry.update()
        }
    }
}