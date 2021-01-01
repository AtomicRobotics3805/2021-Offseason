package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.tuning.SampleMecanumDrive

@TeleOp(name = "Competition OpMode")
class CompTeleOp : BasicTeleOp() {

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        drive = SampleMecanumDrive(hardwareMap)
        drive?.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        waitForStart()

        while (opModeIsActive()) {
            driveMotors()
            telemetryPosition()
        }
    }
}