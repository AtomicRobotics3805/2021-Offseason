package org.firstinspires.ftc.teamcode.subsystems.driving

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator.generateSimpleMotionProfile
import com.acmerobotics.roadrunner.profile.MotionState
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants.constants
import org.firstinspires.ftc.teamcode.Constants.drive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil

@Suppress("unused")
open class Turn(private val angle: Double): AtomicCommand() {
    override val _isDone: Boolean
        get() = turnProfile.duration() > timer.seconds()

    private val timer = ElapsedTime()

    private lateinit var turnProfile: MotionProfile
    private val startPose = drive.poseEstimate

    override fun start() {
        turnProfile = generateSimpleMotionProfile(
                MotionState(drive.poseEstimate.heading, 0.0, 0.0, 0.0),
                MotionState(angle, 0.0, 0.0, 0.0),
                constants.maxAngVel,
                constants.maxAngAccel
        )
    }

    override fun execute() {
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()
        val t = timer.seconds()

        val targetState = turnProfile[t]
        drive.turnController.targetPosition = targetState.x
        val correction = drive.turnController.update(drive.poseEstimate.heading)
        val targetOmega = targetState.v
        val targetAlpha = targetState.a

        drive.setDriveSignal(DriveSignal(
                Pose2d(0.0, 0.0, targetOmega + correction),
                Pose2d(0.0, 0.0, targetAlpha)))

        val newPose = startPose.copy(startPose.x, startPose.y, targetState.x)
        fieldOverlay.setStroke("#4CAF50")
        DashboardUtil.drawRobot(fieldOverlay, newPose)
    }

    override fun done(interrupted: Boolean) {
        drive.setDriveSignal(DriveSignal())
    }

}