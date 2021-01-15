package org.firstinspires.ftc.teamcode.autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.compbot.CompMecanumDrive

class CompAutonomous : LinearOpMode() {
    private var stackSize = ObjectDetection.StackSize.NONE

    override fun runOpMode() {
        telemetry.addLine("Initializing")
        telemetry.update()

        var drive = CompMecanumDrive(hardwareMap)
        var mech = MechanismController(drive)
        var pathManager = PathManager(drive, mech)
        var runtime = ElapsedTime()

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        pathManager.generatePaths()

        stackSize = ObjectDetection.detect(drive)

        telemetry.addLine("Ready")
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