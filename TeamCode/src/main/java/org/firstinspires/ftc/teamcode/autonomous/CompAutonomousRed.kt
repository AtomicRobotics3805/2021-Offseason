package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.teleop.OneDriverTeleOp
import org.firstinspires.ftc.teamcode.teleop.TwoDriverTeleOp

@Autonomous(name="Red Auto")
class CompAutonomousRed : LinearOpMode() {
    private var stackSize = ObjectDetection.StackSize.NONE

    private lateinit var constants: BaseDriveConstants
    private lateinit var drive: MecanumDriveComp
    private lateinit var mech: MechanismController
    private lateinit var pathManager: PathManager
    private val runtime: ElapsedTime = ElapsedTime()
    private val packet = TelemetryPacket()
    private val dashboard = FtcDashboard.getInstance()

    override fun runOpMode() {
        runtime.reset()
        dashboard.telemetryTransmissionInterval = 25

        packet.addLine("Initializing")
        dashboard.sendTelemetryPacket(packet)
        packet.clearLines()

        constants = DriveConstantsComp
        drive = MecanumDriveComp(hardwareMap, constants)
        mech = MechanismController(drive)
        pathManager = PathManager(drive, mech, PathManager.Color.RED)

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        packet.addLine("Started Stack Size Detection")
        dashboard.sendTelemetryPacket(packet)
        packet.clearLines()

        stackSize = ObjectDetection.detect(this)

        while(!(opModeIsActive() || isStopRequested)) {
            packet.addLine("Ready")
            packet.put("Time Elapsed", runtime.seconds())
            packet.put("Detected Stack Size", stackSize.name)
            dashboard.sendTelemetryPacket(packet)
            packet.clearLines()
        }

        runtime.reset()

        packet.addLine("Following Path")
        dashboard.sendTelemetryPacket(packet)
        packet.clearLines()

        pathManager.followPath(stackSize)
        OneDriverTeleOp.startingPose = drive.poseEstimate
        TwoDriverTeleOp.startingPose = drive.poseEstimate

        while(opModeIsActive()) {
            packet.addLine("Path Complete")
            packet.put("Time Elapsed", runtime.seconds())
            dashboard.sendTelemetryPacket(packet)
            packet.clearLines()
        }
    }
}