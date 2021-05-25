package org.firstinspires.ftc.teamcode.util.commands

import com.qualcomm.robotcore.util.ElapsedTime

class TimedCustomCommand(
        private val time: Double,
        private val getDone: () -> Boolean = { true },
        _run: () -> Unit = { },
        _start: () -> Unit = { },
        _done: (interrupted: Boolean) -> Unit = { }
): CustomCommand(getDone, _run, _start, _done) {

    override val _isDone: Boolean
        get() = getDone.invoke() && timer.seconds() > time

    val timer = ElapsedTime()
}