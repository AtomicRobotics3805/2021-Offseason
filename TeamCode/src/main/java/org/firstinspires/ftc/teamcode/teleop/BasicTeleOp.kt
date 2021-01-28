package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive

/*
    Contains simple functionality that will likely be reused in future competitions
    This includes driving and slow modes. It does NOT include contraption usage (arms, pulleys, etc.)
 */

@TeleOp(name = "Competition OpMode")
abstract class BasicTeleOp : LinearOpMode() {

    protected lateinit var drive: BaseMecanumDrive
    protected lateinit var constants: BaseDriveConstants

    private val speedController = SpeedController(TeleOpConstants.speeds)

    fun driveMotors() {
        drive.setWeightedDrivePower(speedController.drivePower)
    }

    fun telemetryPosition() {
        val poseEstimate = drive.poseEstimate
        telemetry.addData("x", poseEstimate.x)
        telemetry.addData("y", poseEstimate.y)
        telemetry.addData("heading", poseEstimate.heading)
        telemetry.update()
    }
}