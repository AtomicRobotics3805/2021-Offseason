package org.firstinspires.ftc.teamcode.util.commands.driving

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.drive.DriveSignal
import org.firstinspires.ftc.teamcode.Constants.drive
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil
import org.firstinspires.ftc.teamcode.util.trajectories.ParallelTrajectory

@Suppress("unused")
class FollowTrajectory(private val trajectory: ParallelTrajectory): AtomicCommand() {
    override val _isDone: Boolean
        get() = MecanumDrive.follower.isFollowing()

    override fun execute() {
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()
        val t = MecanumDrive.follower.elapsedTime()

        MecanumDrive.setDriveSignal(MecanumDrive.follower.update(drive.poseEstimate))

        packet.addLine("following trajectory")
        packet.put("x", drive.poseEstimate.x)
        packet.put("y", drive.poseEstimate.y)
        packet.put("heading", drive.poseEstimate.heading)
        packet.put("time elapsed", t)

        fieldOverlay.setStrokeWidth(1)
        fieldOverlay.setStroke("#4CAF50")
        DashboardUtil.drawSampledPath(fieldOverlay, trajectory.trajectory.path)
        DashboardUtil.drawRobot(fieldOverlay, trajectory.trajectory[t])
        fieldOverlay.setStroke("#3F51B5")

        DashboardUtil.drawPoseHistory(fieldOverlay, MecanumDrive.poseHistory)
    }

    override fun done(interrupted: Boolean) {
        MecanumDrive.setDriveSignal(DriveSignal())
    }
}