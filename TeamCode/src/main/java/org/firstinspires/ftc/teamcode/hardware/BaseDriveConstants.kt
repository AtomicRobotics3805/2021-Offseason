package org.firstinspires.ftc.teamcode.hardware

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import java.lang.reflect.Field

@Suppress("Unused", "MemberVisibilityCanBePrivate")
abstract class BaseDriveConstants {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    val ticksPerRev: Double
        get() = javaClass.getField("_ticksPerRev").getDouble(this)

    val maxRPM: Double
        get() = javaClass.getField("_maxRPM").getDouble(this)

    /*
     * Set runUsingEncoder to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update motorVeloPID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */

    val isRunUsingEncoder: Boolean
        get() = javaClass.getField("_isRunUsingEncoders").getBoolean(this)
    val motorVeloPID: PIDFCoefficients
        get() = javaClass.getField("_motorVeloPID").get(this) as PIDFCoefficients

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from gearRatio.
     */
    val wheelRadius: Double // in
        get() = javaClass.getField("_wheelRadius").getDouble(this)

    val gearRatio: Double // output (wheel) speed / input (motor) speed
        get() = javaClass.getField("_gearRatio").getDouble(this)

    val trackWidth: Double // in
        get() = javaClass.getField("_trackWidth").getDouble(this)


    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    val kV: Double
        get() = javaClass.getField("_kV").getDouble(this)

    val kA: Double
        get() = javaClass.getField("_kA").getDouble(this)

    val kStatic: Double
        get() = javaClass.getField("_kStatic").getDouble(this)

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    val maxVel: Double
        get() = javaClass.getField("_maxVel").getDouble(this)

    val maxAccel: Double
        get() = javaClass.getField("_maxAccel").getDouble(this)

    val maxAngVel: Double
        get() = javaClass.getField("_maxAngVel").getDouble(this)

    val maxAngAccel: Double
        get() = javaClass.getField("_maxAngAccel").getDouble(this)

    val lateralMultiplier: Double
        get() = javaClass.getField("_lateralMultiplier").getDouble(this)

    val driftMultiplier: Double
        get() = javaClass.getField("_driftMultiplier").getDouble(this)

    val driftTurnMultiplier: Double
        get() = javaClass.getField("_driftTurnMultiplier").getDouble(this)


    val translationalPID: PIDCoefficients
        get() = javaClass.getField("_translationalPID").get(this) as PIDCoefficients
    val headingPID: PIDCoefficients
        get() = javaClass.getField("_headingPID").get(this) as PIDCoefficients

    fun encoderTicksToInches(ticks: Double): Double {
        return wheelRadius * 2 * Math.PI * gearRatio * ticks / ticksPerRev
    }

    fun rpmToVelocity(rpm: Double): Double {
        return rpm * gearRatio * 2 * Math.PI * wheelRadius / 60.0
    }

    fun getMotorVelocityF(ticksPerSecond: Double): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond
    }
}