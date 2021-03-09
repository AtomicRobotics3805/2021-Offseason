package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.util.Vector2d
import org.firstinspires.ftc.teamcode.util.toRadians
import kotlin.math.abs
import kotlin.math.atan

/**
 *  Competition Driver Controlled OpMode for 2021
 *
 *  Functionality:
 *      Slow Mode: Whenever the user presses a particular button, the robot changes to a different
 *          speed. Only applicable when the driver is actively controlling the robot
 *      Automatic Turning: Whenever the user presses a different button, the robot begins to turn
 *          towards the tower goals.
 *
 *  Gamepad Map:
 *      Left Stick:
 *          X Axis: Controls strafing (horizontal movement)
 *          Y Axis: Controls forward and backward movement
 *      Right Stick:
 *          X Axis: Controls turning
 *      A Button: Cycles through slow modes
 *      B Button: Causes robot to turn towards the tower goals
 */
@TeleOp(name = "Competition OpMode")
class CompTeleOp : BasicTeleOp(*TeleOpConstants.speeds) {
    private val powerShotPose = listOf(
            Vector2d(72.0, 20.0), Vector2d(72.0, 12.0), Vector2d(72.0, 4.0))
    private val towerPose = Vector2d(72.0,36.0)
    private val startingPose = Pose2d(0.0, 0.0, 0.0)

    private var poseEstimate: Pose2d
        get() = drive.poseEstimate + startingPose
        set(targetPose) {
            drive.poseEstimate = targetPose - startingPose
        }

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        constants = DriveConstantsComp
        drive = MecanumDriveComp(hardwareMap, constants, true)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        waitForStart()

        while (opModeIsActive()) {
            // turning towards tower
            if(gamepad1.b) {
                drive.turnAsync(towerAngle(Vector2d(poseEstimate)))
            }

            drive.update()

            telemetryPosition()
        }
    }

    private fun towerAngle(position: Vector2d): Double {
        return atan(towerPose.y - position.y / towerPose.x - position.x) - 90.0.toRadians
    }

    private fun powerShotAngle(position: Vector2d, num: Int): Double {
        return atan(powerShotPose[num].y - position.y / powerShotPose[num].x - position.x) - 90.0.toRadians
    }
}