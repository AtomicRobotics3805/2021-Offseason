package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad

/*
    CustomGamepad makes it much easier to detect when a button has just been pressed or released
    To use the program, call update(gamepad1) or update(gamepad2) every frame
    Check whether a button is being pressed down using button.down
    Check whether it's just been pressed this frame using button.pressed
    Check whether it's just been released this frame using button.released
 */

class CustomGamepad() {
    var a = Button()
    var b = Button()
    var x = Button()
    var y = Button()

    var dpad_up = Button()
    var dpad_down = Button()
    var dpad_left = Button()
    var dpad_right = Button()

    var left_bumper = Button()
    var right_bumper = Button()

    val left_stick_button = Button()
    val right_stick_button = Button()

    fun update(gamepad : Gamepad) {
        a.pressed = gamepad.a && !a.down
        b.pressed = gamepad.b && !b.down
        x.pressed = gamepad.x && !x.down
        y.pressed = gamepad.y && !y.down

        dpad_up.pressed = gamepad.dpad_up && !dpad_up.down
        dpad_down.pressed = gamepad.dpad_down && !dpad_down.down
        dpad_left.pressed = gamepad.dpad_left && !dpad_left.down
        dpad_right.pressed = gamepad.dpad_right && !dpad_right.down

        left_bumper.pressed = gamepad.left_bumper && !left_bumper.down
        right_bumper.pressed = gamepad.right_bumper && !right_bumper.down

        left_stick_button.pressed = gamepad.left_stick_button && !left_stick_button.down
        right_stick_button.pressed = gamepad.right_stick_button && !right_stick_button.down


        a.released = !gamepad.a && a.down
        b.pressed = !gamepad.b && b.down
        x.pressed = !gamepad.x && x.down
        y.pressed = !gamepad.y && y.down

        dpad_up.pressed = !gamepad.dpad_up && dpad_up.down
        dpad_down.pressed = !gamepad.dpad_down && dpad_down.down
        dpad_left.pressed = !gamepad.dpad_left && dpad_left.down
        dpad_right.pressed = !gamepad.dpad_right && dpad_right.down

        left_bumper.pressed = !gamepad.left_bumper && left_bumper.down
        right_bumper.pressed = !gamepad.right_bumper && right_bumper.down

        left_stick_button.pressed = !gamepad.left_stick_button && left_stick_button.down
        right_stick_button.pressed = !gamepad.right_stick_button && right_stick_button.down


        a.down = gamepad.a
        b.down = gamepad.b
        x.down = gamepad.x
        y.down = gamepad.y

        dpad_up.down = gamepad.dpad_up
        dpad_down.down = gamepad.dpad_down
        dpad_left.down = gamepad.dpad_left
        dpad_right.down = gamepad.dpad_right

        left_bumper.down = gamepad.left_bumper
        right_bumper.down = gamepad.right_bumper

        left_stick_button.down = gamepad.left_stick_button
        right_stick_button.down = gamepad.right_stick_button
    }

    fun getString() : String {
        var set = mutableSetOf<String>()
        if (a.down) set.add("a")
        if (b.down) set.add("b")
        if (x.down) set.add("x")
        if (y.down) set.add("y")

        if (dpad_up.down) set.add("dpad up")
        if (dpad_down.down) set.add("dpad down")
        if (dpad_left.down) set.add("dpad left")
        if (dpad_right.down) set.add("dpad right")

        if (left_bumper.down) set.add("left bumper")
        if (right_bumper.down) set.add("right bumper")

        if (left_stick_button.down) set.add("left stick button")
        if (right_stick_button.down) set.add("right stick button")

        return set.joinToString(", ")
    }
}

class Button {
    var down = false
    var pressed = false
    var released = false
}