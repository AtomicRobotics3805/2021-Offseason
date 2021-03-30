package org.firstinspires.ftc.teamcode.util.commands.delays

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

@Suppress("Unused")
class Delay(private val time: Double): AtomicCommand() {
    override val _isDone: Boolean
        get() = timer.seconds() > time

    private val timer = ElapsedTime()
}