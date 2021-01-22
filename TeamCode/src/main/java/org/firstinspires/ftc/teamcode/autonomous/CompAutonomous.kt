package org.firstinspires.ftc.teamcode.autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.autonomous.MechanismController
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetection
import org.firstinspires.ftc.teamcode.autonomous.PathManager
import org.firstinspires.ftc.teamcode.hardware.compbot.CompMecanumDrive

class CompAutonomous : LinearOpMode() {
    private var stackSize = ObjectDetection.StackSize.NONE

    private lateinit var drive: CompMecanumDrive
    private lateinit var mech: MechanismController
    private lateinit var pathManager: PathManager
    private var runtime: ElapsedTime = ElapsedTime()

    override fun runOpMode() {
        runtime.reset()

        telemetry.addLine("Initializing")
        telemetry.update()

        drive = CompMecanumDrive(hardwareMap)
        mech = MechanismController(drive)
        pathManager = PathManager(drive, mech)

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        stackSize = ObjectDetection.detect(drive)

        telemetry.addLine("Ready")
        telemetry.addData("Time Elapsed", runtime.seconds())
        telemetry.addData("Detected Stack Size", stackSize.name)
        telemetry.update()

        waitForStart()

        runtime.reset()

        telemetry.addLine("Running Path")
        telemetry.update()

        pathManager.followPath(stackSize)

        telemetry.addLine("Path Complete")
        telemetry.addData("Time Elapsed", runtime.seconds())
        telemetry.update()

        while(opModeIsActive()) { }
    }
}