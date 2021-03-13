package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.util.CustomGamepad

/*
    Contains simple functionality that will likely be reused in future competitions
    This includes driving and slow modes. It does NOT include contraption usage (arms, pulleys, etc.)
 */

abstract class BasicTeleOp( vararg speeds: Double) : LinearOpMode() {
    protected lateinit var drive: BaseMecanumDrive
    protected lateinit var constants: BaseDriveConstants

    private val speedController = SpeedController(*speeds)

    fun driveMotors() {
        speedController.update(gamepad1)
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