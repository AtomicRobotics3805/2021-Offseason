package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
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

    protected val packet = TelemetryPacket()
    protected val dashboard: FtcDashboard = FtcDashboard.getInstance()
    private val speedController = SpeedController(*speeds)

    fun driveMotors() {
        speedController.update(gamepad1)
        drive.setWeightedDrivePower(speedController.drivePower)
    }

    fun telemetryPosition() {
        val poseEstimate = drive.poseEstimate
        packet.put("x", poseEstimate.x)
        packet.put("y", poseEstimate.y)
        packet.put("heading", poseEstimate.heading)
        dashboard.sendTelemetryPacket(packet)
        packet.clearLines()
    }
}