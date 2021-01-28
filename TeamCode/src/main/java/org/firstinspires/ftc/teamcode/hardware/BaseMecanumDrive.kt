package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.qualcomm.robotcore.hardware.DcMotor.*
import com.qualcomm.robotcore.hardware.PIDFCoefficients

abstract class BaseMecanumDrive(driveConstants: BaseDriveConstants):
        MecanumDrive(driveConstants.kV, driveConstants.kA, driveConstants.kStatic, driveConstants.trackWidth, driveConstants.trackWidth, driveConstants.lateralMultiplier) {
    abstract var TRANSLATIONAL_PID: PIDCoefficients
    abstract var HEADING_PID: PIDCoefficients
    abstract val VX_WEIGHT: Double
    abstract val VY_WEIGHT: Double
    abstract val OMEGA_WEIGHT: Double
    abstract val POSE_HISTORY_LIMIT: Int
    val lateralMultiplier = driveConstants.lateralMultiplier

    abstract fun setMode(runMode: RunMode?)
    abstract fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior?)
    abstract fun setPIDFCoefficients(runMode: RunMode?, coefficients: PIDFCoefficients)
    abstract fun setWeightedDrivePower(drivePower: Pose2d)
    abstract fun trajectoryBuilder(startPose: Pose2d?): TrajectoryBuilder
    abstract fun trajectoryBuilder(startPose: Pose2d?, reversed: Boolean): TrajectoryBuilder
    abstract fun trajectoryBuilder(startPose: Pose2d?, startHeading: Double): TrajectoryBuilder
    abstract fun turnAsync(angle: Double)
    abstract fun turn(angle: Double)
    abstract fun followTrajectoryAsync(trajectory: Trajectory?)
    abstract fun followTrajectory(trajectory: Trajectory?)
    abstract fun update()
    abstract fun waitForIdle()

    enum class Mode {
        IDLE, TURN, FOLLOW_TRAJECTORY
    }
}