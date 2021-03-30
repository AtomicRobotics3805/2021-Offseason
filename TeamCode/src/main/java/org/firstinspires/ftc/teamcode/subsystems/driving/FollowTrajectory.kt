package org.firstinspires.ftc.teamcode.subsystems.driving

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.Singleton.drive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.DashboardUtil

@Suppress("unused")
class FollowTrajectory(private val trajectory: Trajectory): AtomicCommand() {
    override val _isDone: Boolean
        get() = drive.follower.isFollowing()

    override fun run() {
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()
        val t = drive.follower.elapsedTime()

        drive.setDriveSignal(drive.follower.update(drive.poseEstimate))

        packet.addLine("following trajectory")
        packet.put("x", drive.poseEstimate.x)
        packet.put("y", drive.poseEstimate.y)
        packet.put("heading", drive.poseEstimate.heading)
        packet.put("time elapsed", t)

        fieldOverlay.setStrokeWidth(1)
        fieldOverlay.setStroke("#4CAF50")
        DashboardUtil.drawSampledPath(fieldOverlay, trajectory.path)
        DashboardUtil.drawRobot(fieldOverlay, trajectory[t])
        fieldOverlay.setStroke("#3F51B5")

        DashboardUtil.drawPoseHistory(fieldOverlay, drive.poseHistory)
    }

    override fun done() {
        drive.setDriveSignal(DriveSignal())
    }
}