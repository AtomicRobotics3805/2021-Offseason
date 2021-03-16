package org.firstinspires.ftc.teamcode.autonomous

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
import org.firstinspires.ftc.teamcode.util.toRadians

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

        constants = DriveConstantsComp
        drive = MecanumDriveComp(hardwareMap, constants)
        mech = MechanismController(drive)
        pathManager = PathManager(drive as MecanumDriveComp, mech, PathManager.Color.BLUE)

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)

        telemetry.addLine("Started Stack Size Detection")
        telemetry.update()
        stackSize = ObjectDetection.detect(this)

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

        /*
        var trajectory = drive.trajectoryBuilder(Pose2d())
                .forward(20.0)
                .build()
        drive.followTrajectory(trajectory)*/
        pathManager.followPath(stackSize)
        while(drive.isBusy) {
            drive.update()
        }

        while(opModeIsActive()) {
            telemetry.addLine("Path Complete")
            telemetry.addData("Time Elapsed", runtime.seconds())
            telemetry.update()
        }
    }
}