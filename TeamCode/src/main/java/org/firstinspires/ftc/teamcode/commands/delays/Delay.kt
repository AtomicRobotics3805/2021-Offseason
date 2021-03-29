package org.firstinspires.ftc.teamcode.commands.delays

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.commands.AtomicCommand

class Delay(private val time: Double): AtomicCommand() {
    override val _isDone: Boolean
        get() = timer.seconds() > time

    private val timer = ElapsedTime()
}