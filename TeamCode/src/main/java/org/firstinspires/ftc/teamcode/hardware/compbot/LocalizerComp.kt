package org.firstinspires.ftc.teamcode.hardware.compbot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Encoder
import java.util.*

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
class LocalizerComp(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2, 0.0),  // left
        Pose2d(0.0, -LATERAL_DISTANCE / 2, 0.0),  // right
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
)) {
    private val leftEncoder: Encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "LF"))
    private val rightEncoder: Encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "RF"))
    private val frontEncoder: Encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, "LB"))

    init {
        // FINISHED: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        if (RIGHT_REVERSED) rightEncoder.direction = Encoder.Direction.REVERSE
        if (LEFT_REVERSED) leftEncoder.direction = Encoder.Direction.REVERSE
        if (FRONT_REVERSED) frontEncoder.direction = Encoder.Direction.REVERSE
    }

    override fun getWheelPositions(): List<Double> {
        return listOf(
                encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // FINISHED: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
                encoderTicksToInches(leftEncoder.rawVelocity) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.rawVelocity) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.rawVelocity) * Y_MULTIPLIER
        )
    }

    companion object {
        @JvmField
        var ticksPerRev = 2400.0
        @JvmField
        var wheelRadius = 1.5 // in
        @JvmField
        var gearRatio = 1.0 // output (wheel) speed / input (encoder) speed
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
        var Y_MULTIPLIER = 1.04

        fun encoderTicksToInches(ticks: Double): Double {
            return wheelRadius * 2 * Math.PI * gearRatio * ticks / ticksPerRev
        }
    }
}