package org.firstinspires.ftc.teamcode.util.commands.driving

import com.acmerobotics.roadrunner.drive.DriveSignal
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.trajectories.ParallelTrajectory

@Suppress("unused")
class FollowTrajectory(private val trajectory: ParallelTrajectory): AtomicCommand() {
    override val _isDone: Boolean
        get() = !MecanumDrive.follower.isFollowing()

    override fun start() {
        MecanumDrive.follower.followTrajectory(trajectory.trajectory)
    }

    override fun execute() {
        MecanumDrive.setDriveSignal(MecanumDrive.follower.update(MecanumDrive.poseEstimate))
    }

    override fun done(interrupted: Boolean) {
        MecanumDrive.setDriveSignal(DriveSignal())
    }
}