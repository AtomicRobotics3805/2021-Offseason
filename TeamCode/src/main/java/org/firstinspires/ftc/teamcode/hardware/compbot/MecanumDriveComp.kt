package org.firstinspires.ftc.teamcode.hardware.compbot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.acmerobotics.roadrunner.util.NanoClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.util.DashboardUtil
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil
import java.util.*

/*
* Simple mecanum drive hardware implementation for REV hardware.
*/
@Config
class MecanumDriveComp(val hardwareMap: HardwareMap, constants: BaseDriveConstants, teleOp: Boolean = false, var gamepad: Gamepad? = null) : BaseMecanumDrive(constants, teleOp) {
    override var VX_WEIGHT = 1.0
    override var VY_WEIGHT = 1.0
    override var OMEGA_WEIGHT = 1.0
    override var POSE_HISTORY_LIMIT = 100

    override val clock = NanoClock.system()
    override val turnController = PIDFController(HEADING_PID)

    override val velConstraint = MinVelocityConstraint(listOf(
            AngularVelocityConstraint(constants.maxAngVel),
            MecanumVelocityConstraint(constants.maxVel, constants.trackWidth)))
    override val accelConstraint = ProfileAccelerationConstraint(constants.maxAccel)

    override val follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)

    override val poseHistory: LinkedList<Pose2d> = LinkedList()

    override val batteryVoltageSensor: VoltageSensor = hardwareMap.voltageSensor.iterator().next()

    override val leftFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "LF")
    override val leftRear: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "LB")
    override val rightRear: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "RB")
    override val rightFront: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "RF")
    override val motors = listOf(leftFront, leftRear, rightRear, rightFront)

    override var mode = Mode.IDLE

    private val imu: BNO055IMU

    init {
        dashboard.telemetryTransmissionInterval = 25

        turnController.setInputBounds(0.0, 2 * Math.PI)

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap)

        for (module in hardwareMap.getAll(LynxModule::class.java)) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.AUTO
        }

        // FINISHED: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        imu.initialize(parameters)

        // FINISHED: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        if (constants.isRunUsingEncoder) {
            setMode(RunMode.RUN_USING_ENCODER)
        }
        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE)

        if (constants.isRunUsingEncoder) {
            setPIDFCoefficients(RunMode.RUN_USING_ENCODER, constants.motorVeloPID)
        }

        // FINISHED: reverse any motors using DcMotor.setDirection()
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE

        // FINISHED: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        localizer = LocalizerComp(hardwareMap)
    }

    private val lastError: Pose2d
        get() {
            return when (mode) {
                Mode.FOLLOW_TRAJECTORY -> follower.lastError
                Mode.TURN -> Pose2d(0.0, 0.0, turnController.lastError)
                else -> Pose2d()
            }
        }

    override fun update() {
        updatePoseEstimate()
        val currentPose = poseEstimate
        val lastError = lastError
        poseHistory.add(currentPose)
        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst()
        }
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()
        packet.put("mode", mode)
        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)
        packet.put("xError", lastError.x)
        packet.put("yError", lastError.y)
        packet.put("headingError", lastError.heading)
        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawRobot(fieldOverlay, currentPose)
        dashboard.sendTelemetryPacket(packet)
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

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()
}