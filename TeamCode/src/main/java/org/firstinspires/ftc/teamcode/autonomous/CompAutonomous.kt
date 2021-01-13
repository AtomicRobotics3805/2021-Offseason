package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.compbot.CompMecanumDrive

class CompAutonomous : LinearOpMode() {

    private var drive: CompMecanumDrive? = null
    private var mechanismController: MechanismController? = null

    private var stackSize = ObjectDetection.StackSize.NONE;


    override fun runOpMode() {
        drive = CompMecanumDrive(hardwareMap)
        drive?.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        mechanismController = MechanismController(drive)

        stackSize = ObjectDetection.detect(drive)

        followPath()
    }

    fun followPath() {

    }
}