package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.compbot.CompMecanumDrive

class CompAutonomous : LinearOpMode() {

    var drive: CompMecanumDrive? = null;

    override fun runOpMode() {
        drive?.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }
}