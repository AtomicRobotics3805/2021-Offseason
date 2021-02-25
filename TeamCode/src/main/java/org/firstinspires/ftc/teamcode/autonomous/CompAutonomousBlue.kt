package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp

@Autonomous(name="Blue Auto")
class CompAutonomousBlue : LinearOpMode() {
    private var stackSize = ObjectDetection.StackSize.NONE

    private lateinit var constants: BaseDriveConstants
    private lateinit var drive: BaseMecanumDrive
    private lateinit var mech: MechanismController
    private lateinit var pathManager: PathManager
    private var runtime: ElapsedTime = ElapsedTime()

    override fun runOpMode() {
        runtime.reset()

        telemetry.addLine("Initializing")
        telemetry.update()

        constants = DriveConstantsComp()
        drive = MecanumDriveComp(hardwareMap, constants)
        mech = MechanismController(drive)
        pathManager = PathManager(drive, mech, PathManager.Color.BLUE)

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        stackSize = ObjectDetection.detect(drive)

        while(!(opModeIsActive() || isStopRequested)) {
            telemetry.addLine("Ready")
            telemetry.addData("Time Elapsed", runtime.seconds())
            telemetry.addData("Detected Stack Size", stackSize.name)
            telemetry.update()
        }

        waitForStart()

        runtime.reset()

        telemetry.addLine("Running Path")
        telemetry.update()

        pathManager.followPath(stackSize)

        while(opModeIsActive()) {
            telemetry.addLine("Path Complete")
            telemetry.addData("Time Elapsed", runtime.seconds())
            telemetry.update()
        }
    }
}