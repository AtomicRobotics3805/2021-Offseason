package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.CustomGamepad

/*
    Controls motor speed & slow mode
 */

class SpeedController(private val speeds: List<Double>) {
    private val customGamepad = CustomGamepad()

    private var mode = 0

    var drivePower = Pose2d(0.0, 0.0, 0.0)

    fun update(gamepad: Gamepad) {
        customGamepad.update(gamepad)

        drivePower = Pose2d(
                (-gamepad.left_stick_y).toDouble(),
                (-gamepad.left_stick_x).toDouble(),
                (-gamepad.right_stick_x).toDouble()
        )

        if(customGamepad.a.pressed) {
            mode++
            if(mode >= speeds.size) mode = 0
        }

        drivePower *= speeds[mode]
    }
}