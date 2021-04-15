package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad

/*
    CustomGamepad makes it much easier to detect when a button has just been pressed or released
    To use the program, call update(gamepad1) or update(gamepad2) every frame
    Check whether a button is being pressed down using button.down
    Check whether it's just been pressed this frame using button.pressed
    Check whether it's just been released this frame using button.released
 */


@Suppress("MemberVisibilityCanBePrivate")
class CustomGamepad(private val gamepad: Gamepad) {
    val a = Button("A")
    val b = Button("B")
    val x = Button("X")
    val y = Button("Y")

    val dpad_up = Button("D-Pad Up")
    val dpad_down = Button("D-Pad Down")
    val dpad_left = Button("D-Pad Left")
    val dpad_right = Button("D-Pad Right")

    val left_bumper = Button("Left Bumper")
    val right_bumper = Button("Right Bumper")

    val left_trigger = Trigger("Left Trigger")
    val right_trigger = Trigger("Right Trigger")
    
    val left_stick = JoyStick("Left Stick")
    val right_stick = JoyStick("Right Stick")

    val controls = listOf(a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right,
            left_bumper, right_bumper, left_trigger, right_trigger, left_stick, right_stick)

    fun update(gamepad: Gamepad = this.gamepad) {
        a.update(gamepad.a)
        b.update(gamepad.b)
        x.update(gamepad.x)
        y.update(gamepad.y)

        dpad_up.update(gamepad.dpad_up)
        dpad_down.update(gamepad.dpad_down)
        dpad_left.update(gamepad.dpad_left)
        dpad_right.update(gamepad.dpad_right)

        left_bumper.update(gamepad.left_bumper)
        right_bumper.update(gamepad.right_bumper)
        
        left_trigger.update(gamepad.left_trigger)
        right_trigger.update(gamepad.right_trigger)

        left_stick.update(gamepad.left_stick_x, gamepad.left_stick_y, gamepad.left_stick_button)
        right_stick.update(gamepad.right_stick_x, gamepad.right_stick_y, gamepad.right_stick_button)
    }

    override fun toString(): String {
        val set = mutableSetOf<String>()

        controls.forEach {
            if(it.toString() != "")
                set.add(it.toString())
        }

        return set.joinToString("\n")
    }

    class Button(private val name: String = "Unknown Button") {
        var down = false
        var pressed = false
        var released = false

        fun update(value: Boolean) {
            pressed = value && !down
            released = !value && down
            down = value
        }

        override fun toString(): String {
            val set = mutableSetOf<String>()
            if (down) set.add("Down")
            if (pressed) set.add("Just Pressed")
            if (released) set.add("Just Released")
            return if (set.isNotEmpty()) "$name: ${set.joinToString(", ")}" else ""
        }
    }

    class Trigger(private val name: String = "Unknown Trigger") {
        val down: Boolean
            get() = amount != 0.0f
        var pressed = false
        var released = false
        var amount = 0.0f

        fun update(value: Float) {
            pressed = value != 0.0f && !down
            released = value == 0.0f && down
            amount = value
        }

        override fun toString(): String {
            val set = mutableSetOf<String>()
            if (pressed) set.add("Just Pressed")
            if (released) set.add("Just Released")
            if (amount != 0.0f) set.add("Amount Pressed: $amount")
            return if (set.isNotEmpty()) "$name: ${set.joinToString(", ")}" else ""
        }
    }

    class JoyStick(private val name: String = "Unknown Joystick") {
        val moved: Boolean
            get() = x != 0.0f || y != 0.0f
        var justMoved = false
        var justStopped = false
        var x = 0.0f
        var y = 0.0f
        val button = Button()
        
        fun update(x: Float, y: Float, buttonValue: Boolean) {
            justMoved = x != 0.0f || y != 0.0f && !moved
            justStopped = x == 0.0f && y == 0.0f && moved
            this.x = x
            this.y = y
            button.update(buttonValue)
        }

        override fun toString(): String {
            val set = mutableSetOf<String>()
            if (button.down) set.add("Button Down")
            if (button.pressed) set.add("Button Just Pressed")
            if (button.released) set.add("Button Just Released")
            if (justMoved) set.add("Just Started Moving")
            if (justStopped) set.add("Just Stopped Moving")
            if (set.isNotEmpty() || x != 0.0f || y != 0.0f) {
                set.add("X: $x")
                set.add("Y: $y")
            }
            return if (set.isNotEmpty()) "$name: ${set.joinToString(", ")}" else ""
        }
    }
}