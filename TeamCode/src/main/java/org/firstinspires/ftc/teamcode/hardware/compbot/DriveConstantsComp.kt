package org.firstinspires.ftc.teamcode.hardware.compbot

import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants

/*
* Constants shared between multiple drive types.
*
* FINISHED: Tune or adjust the following constants to fit your robot. Note that the non-final
* fields may also be edited through the dashboard (connect to the robot's WiFi network and
* navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
* adjust them in the dashboard; **config variable changes don't persist between app restarts**.
*
* These are not the only parameters; some are located in the localizer classes, drive base classes,
* and op modes themselves.
*/
class DriveConstantsComp : BaseDriveConstants() {
    /*
     * These are motor constants that should be listed online for your motors.
     */
    override val ticksPerRev = 560.0
    override val maxRPM = 315.0

    /*
     * Set runUsingEncoder to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update motorVeloPID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    override val isRunUsingEncoder = true
    override var motorVeloPID = PIDFCoefficients(0.0, 0.0, 0.0, 12.225)

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from gearRatio.
     */
    override val wheelRadius = 2.0 // in
    override val gearRatio = 1.053 // output (wheel) speed / input (motor) speed
    override val trackWidth = 7.81 // in

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    override val kV = 1.0 / rpmToVelocity(maxRPM)
    override val kA = 0.0
    override val kStatic = 0.0

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. The velocity and
     * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
     * forces acceleration-limited profiling). All distance units are inches.
     */
    override val maxVel = 63.95
    override val maxAccel = 30.0
    override val maxAngVel = Math.toRadians(60.0)
    override val maxAngAccel = Math.toRadians(60.0)

    override val lateralMultiplier = 1.23

    override fun encoderTicksToInches(ticks: Double): Double {
        return wheelRadius * 2 * Math.PI * gearRatio * ticks / ticksPerRev
    }

    override fun rpmToVelocity(rpm: Double): Double {
        return rpm * gearRatio * 2 * Math.PI * wheelRadius / 60.0
    }

    override fun getMotorVelocityF(ticksPerSecond: Double): Double {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond
    }
}