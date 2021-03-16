package org.firstinspires.ftc.teamcode.commands

import com.qualcomm.robotcore.util.ElapsedTime

class Delay(private val time: Double): Command() {
    override val isDone: Boolean
        get() = timer.seconds() > time

    private val timer = ElapsedTime()

    override fun run() { }
}