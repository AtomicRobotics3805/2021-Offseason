package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
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
import org.firstinspires.ftc.teamcode.util.toRadians

@Autonomous(name="Blue Auto")
class CompAutonomousBlue : LinearOpMode() {
    private var stackSize = ObjectDetection.StackSize.NONE

    private lateinit var constants: BaseDriveConstants
    private lateinit var drive: MecanumDriveComp
    private lateinit var mech: MechanismController
    private lateinit var detector: ObjectDetection
    private lateinit var pathManager: PathManager
    private val runtime: ElapsedTime = ElapsedTime()
    private val packet: TelemetryPacket? = null
    private val dashboard: FtcDashboard? = null

    override fun runOpMode() {
        runtime.reset()
        dashboard?.telemetryTransmissionInterval = 25

        packet?.addLine("Initializing")
        dashboard?.sendTelemetryPacket(packet)
        packet?.clearLines()

        constants = DriveConstantsComp
        drive = MecanumDriveComp(hardwareMap, constants)
        mech = MechanismController(drive, false)
        pathManager = PathManager(drive, mech, PathManager.Color.BLUE, this)
        detector = ObjectDetection()
        detector.init(this)

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        packet?.addLine("Started Stack Size Detection")
        dashboard?.sendTelemetryPacket(packet)
        packet?.clearLines()

        stackSize = detector.detect()

        while(!(opModeIsActive() || isStopRequested)) {
            packet?.addLine("Ready")
            packet?.put("Time Elapsed", runtime.seconds())
            packet?.put("Detected Stack Size", stackSize.name)
            //dashboard.sendTelemetryPacket(packet)
            packet?.clearLines()
            stackSize = detector.detect()
        }

        waitForStart()

        runtime.reset()

        packet?.addLine("Following Path")
        dashboard?.sendTelemetryPacket(packet)
        packet?.clearLines()

        /*
        var trajectory = drive.trajectoryBuilder(Pose2d())
                .forward(20.0)
                .build()
        drive.followTrajectory(trajectory)*/
        pathManager.followPath(stackSize)
        while(drive.isBusy) {
            drive.update()
        }
        OneDriverTeleOp.startingPose = drive.poseEstimate
        TwoDriverTeleOp.startingPose = drive.poseEstimate

        while(opModeIsActive()) {
            packet?.addLine("Path Complete")
            packet?.put("Time Elapsed", runtime.seconds())
            dashboard?.sendTelemetryPacket(packet)
            packet?.clearLines()
        }
    }
}