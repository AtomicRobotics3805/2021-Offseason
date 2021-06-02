package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.roadrunner.Encoder

/*
* Sample tracking wheel localizer implementation assuming the standard configuration:
*
*    /--------------\
*    |     ____     |
*    |     ----     |
*    | ||        || |
*    | ||        || |
*    |              |
*    |              |
*    \--------------/
*
*/

@Config
class OdometryLocalizer : ThreeTrackingWheelLocalizer(listOf(
        Pose2d(0.0, Constants.LATERAL_DISTANCE / 2, 0.0),  // left
        Pose2d(0.0, -Constants.LATERAL_DISTANCE / 2, 0.0),  // right
        Pose2d(Constants.FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
)) {
    val leftEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "LF"))
    val rightEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "RF"))
    val frontEncoder: Encoder = Encoder(opMode.hardwareMap.get(DcMotorEx::class.java, "LB"))

    init {
        // FINISHED: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        if (Constants.RIGHT_REVERSED) rightEncoder.direction = Encoder.Direction.REVERSE
        if (Constants.LEFT_REVERSED) leftEncoder.direction = Encoder.Direction.REVERSE
        if (Constants.FRONT_REVERSED) frontEncoder.direction = Encoder.Direction.REVERSE
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * Constants.X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * Constants.X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.currentPosition.toDouble()) * Constants.Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // FINISHED: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
                encoderTicksToInches(leftEncoder.rawVelocity) * Constants.X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.rawVelocity) * Constants.X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.rawVelocity) * Constants.Y_MULTIPLIER
        )
    }

    private fun encoderTicksToInches(ticks: Double): Double {
        return Constants.WHEEL_RADIUS * 2 * Math.PI * Constants.GEAR_RATIO * ticks / Constants.TICKS_PER_REV
    }

    object Constants {
        @JvmField
        var TICKS_PER_REV = 2400.0
        @JvmField
        var WHEEL_RADIUS = 1.5 // in
        @JvmField
        var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField
        var LATERAL_DISTANCE = 17.3 // in; distance between the left and right wheels
        @JvmField
        var FORWARD_OFFSET = -3.6 // in; offset of the lateral wheel
        @JvmField
        var LEFT_REVERSED = true
        @JvmField
        var RIGHT_REVERSED = true
        @JvmField
        var FRONT_REVERSED = true
        @JvmField
        var X_MULTIPLIER = 1.04
        @JvmField
        var Y_MULTIPLIER = 1.062
    }
}