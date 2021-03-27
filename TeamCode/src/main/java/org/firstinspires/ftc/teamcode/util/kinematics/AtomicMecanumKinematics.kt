package org.firstinspires.ftc.teamcode.util.kinematics

import com.acmerobotics.roadrunner.geometry.Pose2d

object AtomicMecanumKinematics {
    @JvmStatic
    fun robotToWheelVelocities(
            robotVel: Pose2d,
            trackWidth: Double,
            wheelBase: Double = trackWidth,
            lateralMultiplier: Double = 1.0,
            driftMultiplier: Double = 1.0
    ): List<Double> {
        val k = (trackWidth + wheelBase) / 2.0
        return listOf(
                robotVel.x - lateralMultiplier * robotVel.y / driftMultiplier - k * robotVel.heading,
                robotVel.x + lateralMultiplier * robotVel.y * driftMultiplier - k * robotVel.heading,
                robotVel.x - lateralMultiplier * robotVel.y / driftMultiplier + k * robotVel.heading,
                robotVel.x + lateralMultiplier * robotVel.y * driftMultiplier + k * robotVel.heading
        )
    }

    @JvmStatic
    fun robotToWheelAccelerations(
            robotAccel: Pose2d,
            trackWidth: Double,
            wheelBase: Double = trackWidth,
            lateralMultiplier: Double = 1.0,
            driftMultiplier: Double = 1.0
    ) =
            robotToWheelVelocities(
                    robotAccel,
                    trackWidth,
                    wheelBase,
                    lateralMultiplier,
                    driftMultiplier
            )

    @JvmStatic
    fun wheelToRobotVelocities(
            wheelVelocities: List<Double>,
            trackWidth: Double,
            wheelBase: Double = trackWidth,
            lateralMultiplier: Double = 1.0,
            driftMultiplier: Double = 1.0
    ): Pose2d {
        val k = (trackWidth + wheelBase) / 2.0
        val (frontLeft, rearLeft, rearRight, frontRight) = wheelVelocities
        return Pose2d(
                wheelVelocities.sum(),
                ((rearLeft + frontRight) * driftMultiplier - (frontLeft + rearRight) / driftMultiplier) / lateralMultiplier,
                (rearRight + frontRight - frontLeft - rearLeft) / k
        ) * 0.25
    }
}