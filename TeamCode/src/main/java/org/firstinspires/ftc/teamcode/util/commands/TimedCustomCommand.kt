package org.firstinspires.ftc.teamcode.util.commands

import com.qualcomm.robotcore.util.ElapsedTime

class TimedCustomCommand(
        private val time: Double,
        getDone: () -> Boolean = { true },
        _run: () -> Unit = { },
        _start: () -> Unit = { },
        _done: () -> Unit = { }
): CustomCommand(getDone, _run, _start, _done) {

    override val _isDone: Boolean
        get() = super._isDone && timer.seconds() > time

    private val timer = ElapsedTime()
}