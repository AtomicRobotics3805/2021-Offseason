package org.firstinspires.ftc.teamcode.hardware.compbot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
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
@Config
object DriveConstantsComp : BaseDriveConstants() {
    init {
        /*
         * These are motor constants that should be listed online for your motors.
         */
        ticksPerRev = 560.0
        maxRPM = 315.0

        /*
         * Set runUsingEncoder to true to enable built-in hub velocity control using drive encoders.
         * Set this flag to false if drive encoders are not present and an alternative localization
         * method is in use (e.g., tracking wheels).
         *
         * If using the built-in motor velocity PID, update motorVeloPID with the tuned coefficients
         * from DriveVelocityPIDTuner.
         */
        motorVeloPID = PIDFCoefficients(0.0, 0.0, 0.0, 12.225)
        isRunUsingEncoder = false

        /*
         * These are physical constants that can be determined from your robot (including the track
         * width; it will be tune empirically later although a rough estimate is important). Users are
         * free to chose whichever linear distance unit they would like so long as it is consistently
         * used. The default values were selected with inches in mind. Road runner uses radians for
         * angular distances although most angular parameters are wrapped in Math.toRadians() for
         * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from gearRatio.
         */
         wheelRadius = 2.0 // in
         gearRatio = 0.5 // output (wheel) speed / input (motor) speed
         trackWidth = 23.0 // in
        
        /*
         * These are the feedforward parameters used to model the drive motor behavior. If you are using
         * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
         * motor encoders or have elected not to use them for velocity control, these values should be
         * empirically tuned.
         */

        kV = 0.0245
        kA = 0.0035
        kStatic = 0.01
        
        /*
         * These values are used to generate the trajectories for you robot. To ensure proper operation,
         * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
         * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
         * small and gradually increase them later after everything is working. The velocity and
         * acceleration values are required, and the jerk values are optional (setting a jerk of 0.0
         * forces acceleration-limited profiling). All distance units are inches.
         */
        maxVel = 40.0
        maxAccel = 45.0
        maxAngVel = Math.toRadians(60.0)
        maxAngAccel = Math.toRadians(60.0)

        lateralMultiplier = 1.0

        translationalPID = PIDCoefficients(8.0, 0.0, 0.0)
        headingPID = PIDCoefficients(8.0, 0.0, 0.0)
    }
}