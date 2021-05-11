@file:Suppress("MemberVisibilityCanBePrivate", "unused")
package org.firstinspires.ftc.teamcode.subsystems.driving

import android.graphics.Path
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.drive.DriveSignal
import com.acmerobotics.roadrunner.drive.MecanumDrive
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.hardware.DcMotor.RunMode
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CustomCommand
import org.firstinspires.ftc.teamcode.util.commands.Subsystem
import org.firstinspires.ftc.teamcode.util.inchesToMm
import org.firstinspires.ftc.teamcode.util.kinematics.AtomicMecanumKinematics
import org.firstinspires.ftc.teamcode.util.roadrunner.DashboardUtil
import org.firstinspires.ftc.teamcode.util.roadrunner.LynxModuleUtil
import org.firstinspires.ftc.teamcode.util.trajectories.ParallelTrajectory
import java.util.*
import kotlin.math.abs


/*
 * These are motor constants that should be listed online for your motors.
 */
@JvmField
var TICKS_PER_REV = 560.0
@JvmField
var MAX_RPM = 315.0

/*
 * Set runUsingEncoder to true to enable built-in hub velocity control using drive encoders.
 * Set this flag to false if drive encoders are not present and an alternative localization
 * method is in use (e.g., tracking wheels).
 *
 * If using the built-in motor velocity PID, update motorVeloPID with the tuned coefficients
 * from DriveVelocityPIDTuner.
 */
@JvmField
var MOTOR_VELO_PID = PIDFCoefficients(0.0, 0.0, 0.0, 12.225)
@JvmField
var IS_RUN_USING_ENCODER = false

/*
 * These are physical constants that can be determined from your robot (including the track
 * width; it will be tune empirically later although a rough estimate is important). Users are
 * free to chose whichever linear distance unit they would like so long as it is consistently
 * used. The default values were selected with inches in mind. Road runner uses radians for
 * angular distances although most angular parameters are wrapped in Math.toRadians() for
 * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from gearRatio.
 */
@JvmField
var WHEEL_RADIUS = 2.0 // in
@JvmField
var GEAR_RATIO = 0.5 // output (wheel) speed / input (motor) speed
@JvmField
var TRACK_WIDTH = 23.0 // in

/*
 * These are the feedforward parameters used to model the drive motor behavior. If you are using
 * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
 * motor encoders or have elected not to use them for velocity control, these values should be
 * empirically tuned.
 */

@JvmField
var kV = 0.0245
@JvmField
var kA = 0.0035
@JvmField
var kStatic = 0.01

/*
 * These values are used to generate the trajectories for you robot. To ensure proper operation,
 * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
 * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
 * small and gradually increase them later after everything is working. The velocity and
 * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
 * forces acceleration-limited profiling). All distance units are inches.
 */
@JvmField
var MAX_VEL = 40.0
@JvmField
var MAX_ACCEL = 45.0
@JvmField
var MAX_ANG_VEL = Math.toRadians(60.0)
@JvmField
var MAX_ANG_ACCEL = Math.toRadians(60.0)

/*
 * These values are used solely with Mecanum Drives to adjust the kinematics functions that
 * translate robot velocity to motor speeds. The only way to find these values is to tune and
 * adjust them until they seem about right. They should be close to 1.0.
 */

@JvmField
var LATERAL_MULTIPLIER = 1.0

@JvmField
var DRIFT_MULTIPLIER = 1.0
@JvmField
var DRIFT_TURN_MULTIPLIER = 1.0

/*
 * These coefficients are used to adjust your location and heading when they don't match up with
 * where you should be. The only way to get these values is through tuning, but generally P=8
 * I=0 and D=0 are reasonable starting points.
 */

@JvmField
var TRANSLATIONAL_PID = PIDCoefficients(8.0, 0.0, 0.0)
@JvmField
var HEADING_PID = PIDCoefficients(8.0, 0.0, 0.0)

@JvmField
var VX_WEIGHT = 1.0
@JvmField
var VY_WEIGHT = 1.0
@JvmField
var OMEGA_WEIGHT = 1.0
@JvmField
var POSE_HISTORY_LIMIT = 100

@JvmField
var VUFORIA_KEY = " --- YOUR NEW VUFORIA KEY GOES HERE  --- "

@JvmField
var CAMERA_FORWARD_DISPLACEMENT = 4.0.inchesToMm.toFloat() // eg: Camera is 4 Inches in front of robot-center
@JvmField
var CAMERA_VERTICAL_DISPLACEMENT = 8.0.inchesToMm.toFloat() // eg: Camera is 8 Inches above ground
@JvmField
var CAMERA_LEFT_DISPLACEMENT = 0f // eg: Camera is ON the robot's center line

/*
* Simple mecanum drive hardware implementation for REV hardware.
*/
@Config
object MecanumDrive : MecanumDrive(kA, kStatic, kV, TRACK_WIDTH), Subsystem {
    val cameraLocationOnRobot: OpenGLMatrix
            get() = OpenGLMatrix
            .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
            .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY, AngleUnit.DEGREES, 90f, 90f, 0f))

    val velConstraint = MinVelocityConstraint(listOf(
            AngularVelocityConstraint(MAX_ANG_VEL),
            MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)))
    val accelConstraint = ProfileAccelerationConstraint(MAX_ACCEL)

    val follower = HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
            Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5)
    val turnController = PIDFController(HEADING_PID)

    val poseHistory: LinkedList<Pose2d> = LinkedList()

    lateinit var leftFront: DcMotorEx
    lateinit var leftRear: DcMotorEx
    lateinit var rightRear: DcMotorEx
    lateinit var rightFront: DcMotorEx
    lateinit var motors: List<DcMotorEx>
    lateinit var batteryVoltageSensor: VoltageSensor
    private lateinit var imu: BNO055IMU

    private val dashboard = FtcDashboard.getInstance()
    lateinit var telemetry: MultipleTelemetry

    private lateinit var hardwareMap: HardwareMap

    val driverSpeeds = listOf(0.1, 0.4, 1.0)
    var driverSpeedIndex = 0
    val driverSpeed: Double
        get() = driverSpeeds[driverSpeedIndex]

    var trajectory: ParallelTrajectory? = null

    val switchSpeed: AtomicCommand
        get() = CustomCommand(_start = {
            driverSpeedIndex++
            if (driverSpeedIndex >= driverSpeeds.size)
                driverSpeedIndex = 0
        })

    fun initialize() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next()

        dashboard.telemetryTransmissionInterval = 25
        telemetry = MultipleTelemetry(opMode.telemetry, dashboard.telemetry)

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

        if (IS_RUN_USING_ENCODER) {
            setMode(RunMode.STOP_AND_RESET_ENCODER)
            setMode(RunMode.RUN_USING_ENCODER)
        }
        setZeroPowerBehavior(ZeroPowerBehavior.BRAKE)

        if (IS_RUN_USING_ENCODER) {
            setPIDFCoefficients(RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID)
        }

        // FINISHED: reverse any motors using DcMotor.setDirection()
        leftRear.direction = DcMotorSimple.Direction.REVERSE
        leftFront.direction = DcMotorSimple.Direction.REVERSE

        // FINISHED: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
        localizer = OdometryLocalizer
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
        telemetry.addData("x", currentPose.x)
        telemetry.addData("y", currentPose.y)
        telemetry.addData("heading", currentPose.heading)
        telemetry.update()
        fieldOverlay.setStroke("#3F51B5")
        DashboardUtil.drawRobot(fieldOverlay, currentPose)
        dashboard.sendTelemetryPacket(packet)
    }

    override val rawExternalHeading: Double
        get() = imu.angularOrientation.firstAngle.toDouble()

    fun driverControlled(gamepad: Gamepad): AtomicCommand = DriverControlled(gamepad)
    fun followTrajectory(trajectory: Trajectory): AtomicCommand = FollowTrajectory(trajectory)
    fun turn(angle: Double): AtomicCommand = Turn(angle)
    fun turnRelative(angle: Double): AtomicCommand = Turn(angle + poseEstimate.heading)

    fun encoderTicksToInches(ticks: Double): Double {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
    }

    fun rpmToVelocity(rpm: Double): Double {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0
    }

    fun getMotorVelocityF(ticksPerSecond: Double): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond
    }

    fun trajectoryBuilder(startPose: Pose2d) =
            TrajectoryBuilder(startPose, false, velConstraint, accelConstraint)

    fun trajectoryBuilder(startPose: Pose2d, reversed: Boolean) =
            TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint)

    fun trajectoryBuilder(startPose: Pose2d, startHeading: Double) =
            TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint)

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
            wheelPositions.add(encoderTicksToInches(motor.currentPosition.toDouble()))
        }
        return wheelPositions
    }

    override fun getWheelVelocities(): List<Double> {
        val wheelVelocities: MutableList<Double> = ArrayList()
        for (motor in motors) {
            wheelVelocities.add(encoderTicksToInches(motor.velocity))
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
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER,
                DRIFT_MULTIPLIER,
                DRIFT_TURN_MULTIPLIER
        )
        val accelerations = AtomicMecanumKinematics.robotToWheelAccelerations(
                driveSignal.accel,
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER,
                DRIFT_MULTIPLIER,
                DRIFT_TURN_MULTIPLIER
        )
        val powers = Kinematics.calculateMotorFeedforward(velocities, accelerations, kV, kA, kStatic)
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }

    override fun setDrivePower(drivePower: Pose2d) {
        val powers = AtomicMecanumKinematics.robotToWheelVelocities(
                drivePower,
                TRACK_WIDTH,
                TRACK_WIDTH,
                LATERAL_MULTIPLIER,
                DRIFT_MULTIPLIER,
                DRIFT_TURN_MULTIPLIER
        )
        setMotorPowers(powers[0], powers[1], powers[2], powers[3])
    }
}