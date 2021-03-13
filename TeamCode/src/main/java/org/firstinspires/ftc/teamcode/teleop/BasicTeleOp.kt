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

abstract class BasicTeleOp(private val speeds: List<Double>) : LinearOpMode() {
    private val customGamepad = CustomGamepad()

    private var mode = 0

    var drivePower = Pose2d(0.0, 0.0, 0.0)

    protected lateinit var drive: BaseMecanumDrive
    protected lateinit var constants: BaseDriveConstants

    private val speedController = SpeedController(TeleOpConstants.speeds)

    fun driveMotors(gamepad: Gamepad) {
        speedControl(gamepad)
        drive.setWeightedDrivePower(drivePower)
        telemetry.addData("Drive Power", drivePower)
    }

    fun telemetryPosition() {
        val poseEstimate = drive.poseEstimate
        telemetry.addData("x", poseEstimate.x)
        telemetry.addData("y", poseEstimate.y)
        telemetry.addData("heading", poseEstimate.heading)
        telemetry.update()
    }

    private fun speedControl(gamepad: Gamepad) {
        customGamepad.update(gamepad)

        drivePower = Pose2d(
                (gamepad.left_stick_y).toDouble(),
                (gamepad.left_stick_x).toDouble(),
                (gamepad.right_stick_x).toDouble()
        )

        if(customGamepad.a.pressed) {
            mode++
            if(mode >= speeds.size) mode = 0
        }

        drivePower *= speeds[mode]
    }
}