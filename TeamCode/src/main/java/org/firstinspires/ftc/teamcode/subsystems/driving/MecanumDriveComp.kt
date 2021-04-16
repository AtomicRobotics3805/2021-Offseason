package org.firstinspires.ftc.teamcode.subsystems.driving

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
import org.firstinspires.ftc.teamcode.Constants.constants
import org.firstinspires.ftc.teamcode.hardware.compbot.LocalizerComp
import org.firstinspires.ftc.teamcode.util.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil
import org.firstinspires.ftc.teamcode.util.roadrunner.LynxModuleUtil
import java.util.*

/*
* Simple mecanum drive hardware implementation for REV hardware.
*/
@Suppress("unused")
@Config
object MecanumDriveComp : BaseMecanumDrive(constants) {
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

    override lateinit var batteryVoltageSensor: VoltageSensor

    override lateinit var leftFront: DcMotorEx
    override lateinit var leftRear: DcMotorEx
    override lateinit var rightRear: DcMotorEx
    override lateinit var rightFront: DcMotorEx
    override lateinit var motors: List<DcMotorEx>

    private lateinit var imu: BNO055IMU

    private lateinit var hardwareMap: HardwareMap

    override val driverSpeeds = listOf(0.1, 0.4, 1.0)
    override var driverSpeedIndex = 0
    override val driverSpeed: Double
        get() = driverSpeeds[driverSpeedIndex]

    override fun initialize() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

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

        leftFront = hardwareMap.get(DcMotorEx::class.java, "LF")
        leftRear = hardwareMap.get(DcMotorEx::class.java, "LB")
        rightRear = hardwareMap.get(DcMotorEx::class.java, "RB")
        rightFront = hardwareMap.get(DcMotorEx::class.java, "RF")

        motors = listOf(leftFront, leftRear, rightRear, rightFront)

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

    override fun periodic() {
        updatePoseEstimate()
        val currentPose = poseEstimate
        poseHistory.add(currentPose)
        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst()
        }
        val packet = TelemetryPacket()
        val fieldOverlay = packet.fieldOverlay()
        packet.put("x", currentPose.x)
        packet.put("y", currentPose.y)
        packet.put("heading", currentPose.heading)
        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawRobot(fieldOverlay, currentPose)
        dashboard.sendTelemetryPacket(packet)
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()
}