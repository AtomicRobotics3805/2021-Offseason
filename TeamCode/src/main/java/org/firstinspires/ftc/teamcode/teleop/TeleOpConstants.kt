package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.config.Config

@Config
object TeleOpConstants {
    @JvmField
    val speeds = doubleArrayOf(1.0, 0.5)
    // the amount of time after a ring gets shot before the servos start retracting
    const val SERVO_BACK_TIME = 0.3 // seconds
    // the minimum amount of time between ring shots
    const val SHOOT_TIME = SERVO_BACK_TIME * 2
}