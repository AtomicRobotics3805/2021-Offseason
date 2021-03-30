package org.firstinspires.ftc.teamcode.util.hardware

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.TrajectoryFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.teamcode.util.trajectories.ParallelTrajectory
import org.firstinspires.ftc.teamcode.util.kinematics.AtomicMecanumKinematics
import java.util.*
import kotlin.math.abs

abstract class BaseMecanumDrive(val constants: BaseDriveConstants):
        MecanumDrive(constants.kV, constants.kA, constants.kStatic, constants.trackWidth, constants.trackWidth, constants.lateralMultiplier) {
    var TRANSLATIONAL_PID = constants.translationalPID
    var HEADING_PID = constants.headingPID
    abstract val VX_WEIGHT: Double
    abstract val VY_WEIGHT: Double
    abstract val OMEGA_WEIGHT: Double
    abstract val POSE_HISTORY_LIMIT: Int

    protected abstract val velConstraint: TrajectoryVelocityConstraint
    protected abstract val accelConstraint: TrajectoryAccelerationConstraint
    abstract val follower: TrajectoryFollower
    abstract val poseHistory: LinkedList<Pose2d>

    protected abstract val leftFront: DcMotorEx
    protected abstract val leftRear: DcMotorEx
    protected abstract val rightRear: DcMotorEx
    protected abstract val rightFront: DcMotorEx
    protected abstract val motors: List<DcMotorEx>

    protected abstract val clock: NanoClock
    abstract val turnController: PIDFController
    protected abstract val batteryVoltageSensor: VoltageSensor
    protected var turnProfile: MotionProfile? = null
    protected var turnStart = 0.0
    protected val dashboard: FtcDashboard = FtcDashboard.getInstance()

    protected var lastPoseOnTurn: Pose2d? = null

    var trajectory: ParallelTrajectory? = null
    
    fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, false, velConstraint, accelConstraint)
    }

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint)
    }

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double): TrajectoryBuilder {
        return TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint)
    }

    fun setMode(runMode: RunMode?) {
        for (motor in motors) {
            motor.mode = runMode
        }
    }

    fun setZeroPowerBehavior(zeroPowerBehavior: ZeroPowerBehavior?) {
        for (motor in motors) {
            motor.zeroPowerBehavior = zeroPowerBehavior
        }
    }

    fun setPIDFCoefficients(runMode: RunMode?, coefficients: PIDFCoefficients) {
        val compensatedCoefficients = PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.voltage
        )
        for (motor in motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients)
        }
    }

    fun setWeightedDrivePower(drivePower: Pose2d) {
        var vel = drivePower
        if ((abs(drivePower.x) + abs(drivePower.y)
                        + abs(drivePower.heading)) > 1) {
            // re-normalize the powers according to the weights
            val denom = VX_WEIGHT * abs(drivePower.x) + VY_WEIGHT * abs(drivePower.y) + OMEGA_WEIGHT * abs(drivePower.heading)
            vel = Pose2d(
                    VX_WEIGHT * drivePower.x,
                    VY_WEIGHT * drivePower.y,
                    OMEGA_WEIGHT * drivePower.heading
            ).div(denom)
        }
        setDrivePower(vel)
    }

    override fun getWheelPositions(): List<Double> {
        val wheelPositions: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelPositions.add(constants.encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelVelocities.add(constants.encoderTicksToInches(motor.velocity))
        }
        return wheelVelocities
    }

    override fun setMotorPowers(frontLeft: Double, rearLeft: Double, rearRight: Double, frontRight: Double) {
        leftFront.power = frontLeft
        leftRear.power = rearLeft
        rightRear.power = rearRight
        rightFront.power = frontRight
    }

    override fun setDriveSignal(driveSignal: DriveSignal) {
        val velocities = AtomicMecanumKinematics.robotToWheelVelocities(
                driveSignal.vel,
                constants.trackWidth,
                constants.trackWidth,
                constants.lateralMultiplier,
                constants.driftMultiplier,
                constants.driftTurnMultiplier
        )
        val accelerations = AtomicMecanumKinematics.robotToWheelAccelerations(
                driveSignal.accel,
                constants.trackWidth,
                constants.trackWidth,
                constants.lateralMultiplier,
                constants.driftMultiplier,
                constants.driftTurnMultiplier
        )
        val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, constants.kV, constants.kA, constants.kStatic)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val powers = AtomicMecanumKinematics.robotToWheelVelocities(
                drivePower,
                1.0,
                1.0,
                constants.lateralMultiplier,
                constants.driftMultiplier,
                constants.driftTurnMultiplier
        )
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    abstract fun update()
}