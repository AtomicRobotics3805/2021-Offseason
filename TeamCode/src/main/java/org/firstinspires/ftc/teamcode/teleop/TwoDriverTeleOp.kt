package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.autonomous.MechanismController
import org.firstinspires.ftc.teamcode.autonomous.PathManager
import org.firstinspires.ftc.teamcode.hardware.compbot.DriveConstantsComp
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.util.CustomGamepad
import org.firstinspires.ftc.teamcode.util.Vector2d
import org.firstinspires.ftc.teamcode.util.toRadians
import kotlin.jvm.Throws
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
 *  Down aligns with goal, up moves it to position that transfers
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
@TeleOp(name = "Two Driver TeleOp")
class TwoDriverTeleOp : BasicTeleOp(*TeleOpConstants.speeds) {
    private val powerShotPose = listOf(
            Vector2d(72.0, 20.0), Vector2d(72.0, 12.0), Vector2d(72.0, 4.0))
    private val towerPose = Vector2d(72.0,36.0)
    private lateinit var mech: MechanismController
    private val ringServoTimer = ElapsedTime()
    private val shootRingTimer = ElapsedTime()
    private val raiseArmTimer = ElapsedTime()
    private val customGamepad1 = CustomGamepad()
    private val customGamepad2 = CustomGamepad()
    private var aboutToRaiseArm = false
    private var aboutToCloseClaw = false
    private var aboutToOpenClaw = false

    @Throws(InterruptedException::class)
    override fun runOpMode() {
        dashboard.telemetryTransmissionInterval = 25
        constants = DriveConstantsComp
        drive = MecanumDriveComp(hardwareMap, constants, true, gamepad1)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        mech = MechanismController(drive)
        drive.poseEstimate = startingPose
        waitForStart()

        while (opModeIsActive()) {
            customGamepad1.update(gamepad1)
            customGamepad2.update(gamepad2)
            // turning towards tower
            if(customGamepad1.b.pressed) {
                drive.turnAsync(towerAngle(Vector2d(drive.poseEstimate)) - drive.poseEstimate.heading)
            }

            if(customGamepad2.a.pressed) {
                mech.switchIntake()
            }

            if(customGamepad2.b.pressed) {
                mech.switchShooter()
            }

            if(customGamepad2.x.pressed) {
                mech.reverseIntake()
            }

            if(customGamepad2.dpad_up.pressed) {
                aboutToRaiseArm = true
                raiseArmTimer.reset()
                mech.closeClawManually()
            }

            if(customGamepad2.dpad_down.pressed) {
                mech.alignGoal()
            }

            if(customGamepad2.dpad_left.pressed) {
                mech.raiseArm()
                aboutToOpenClaw = true
            }

            if(gamepad2.left_trigger > 0.1) {
                mech.raiseArmManually()
                mech.targetPos = drive.wobbleArm.currentPosition
            }

            else if(gamepad2.right_trigger > 0.1) {
                mech.lowerArmManually()
                mech.targetPos = drive.wobbleArm.currentPosition
            }

            else if(drive.wobbleArm.mode != DcMotor.RunMode.RUN_TO_POSITION) {
                mech.stopArmManually()
            }

            else {
                mech.targetPos = drive.wobbleArm.currentPosition
            }

            if(customGamepad2.left_bumper.pressed) {
                mech.openClawManually()
            }

            if(customGamepad2.left_bumper.pressed) {
                mech.closeClawManually()
            }

            if(aboutToRaiseArm && raiseArmTimer.seconds() > 1.0) {
                aboutToRaiseArm = false
                mech.raiseArmHigh()
            }

            if(aboutToOpenClaw && !drive.wobbleArm.isBusy) {
                aboutToOpenClaw = false
                mech.openClawManually()
            }

            if(aboutToCloseClaw && !drive.wobbleArm.isBusy) {
                aboutToCloseClaw = false
                mech.closeClawManually()
            }

            if(customGamepad2.y.pressed && shootRingTimer.seconds() > TeleOpConstants.SHOOT_TIME) {
                mech.shootRing()
                ringServoTimer.reset()
                shootRingTimer.reset()
            }

            if(mech.areServosExtended() && ringServoTimer.seconds() > TeleOpConstants.SERVO_BACK_TIME) {
                mech.retractShooterServos()
            }

            drive.update()

            telemetryPosition()
        }
    }

    private fun towerAngle(position: Vector2d): Double {
        return (position - towerPose angleBetween Vector2d(1.0, 0.0)) + 5.0.toRadians + PathManager.OFFSET.toRadians
    }

    private fun powerShotAngle(position: Vector2d, num: Int): Double {
        return (position - powerShotPose[num] angleBetween Vector2d(1.0, 0.0)) + PathManager.OFFSET.toRadians
    }

    companion object {
        var startingPose = Pose2d(0.0, 0.0, 0.0)
    }
}