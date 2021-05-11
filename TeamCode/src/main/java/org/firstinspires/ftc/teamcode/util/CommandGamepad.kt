package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler

@Suppress("MemberVisibilityCanBePrivate")
class CommandGamepad(qualcommGamepad: Gamepad) {

    enum class TriggerType {
        DOWN,
        PRESSED,
        RELEASED
    }

    private val gamepad = CustomGamepad(qualcommGamepad)

    val a = CommandButton(gamepad.a)
    val b = CommandButton(gamepad.b)
    val x = CommandButton(gamepad.x)
    val y = CommandButton(gamepad.y)

    val dpadUp = CommandButton(gamepad.dpadUp)
    val dpadDown = CommandButton(gamepad.dpadDown)
    val dpadLeft = CommandButton(gamepad.dpadLeft)
    val dpadRight = CommandButton(gamepad.dpadRight)

    val leftBumper = CommandButton(gamepad.leftBumper)
    val rightBumper = CommandButton(gamepad.rightBumper)

    private val controls = listOf(a, b, x, y, dpadUp, dpadDown, dpadLeft, dpadRight,
            leftBumper, rightBumper)

    fun update() {
        gamepad.update()
        controls.forEach { it.update() }
    }

    class CommandButton(button: CustomGamepad.Button) {
        val down = CommandButtonTrigger(button, TriggerType.DOWN)
        val pressed = CommandButtonTrigger(button, TriggerType.PRESSED)
        val released = CommandButtonTrigger(button, TriggerType.RELEASED)

        fun update() {
            down.update()
            pressed.update()
            released.update()
        }

        class CommandButtonTrigger(private val button: CustomGamepad.Button,
                                   private val triggerType: TriggerType) {
            var command: (() -> AtomicCommand)? = null

            fun update() {
                if(command != null && ((triggerType == TriggerType.DOWN && button.down) ||
                                (triggerType == TriggerType.PRESSED && button.pressed) || 
                                (triggerType == TriggerType.RELEASED && button.released))) {
                    CommandScheduler.commands += command!!.invoke()
                }
            }
        }
    }
}