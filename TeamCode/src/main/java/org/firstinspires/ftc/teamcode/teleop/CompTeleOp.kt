package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp

@TeleOp(name = "Competition OpMode")
class CompTeleOp : BasicTeleOp() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        constants = DriveConstantsComp()
        drive = MecanumDriveComp(hardwareMap, constants)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        waitForStart()

        while (opModeIsActive()) {
            driveMotors()
            telemetryPosition()
        }
    }
}