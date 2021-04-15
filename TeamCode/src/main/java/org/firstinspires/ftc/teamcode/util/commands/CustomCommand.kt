package org.firstinspires.ftc.teamcode.util.commands

open class CustomCommand(
        private val getDone: () -> Boolean = { true },
        private val _execute: () -> Unit = { },
        private val _start: () -> Unit = { },
        private val _done: (interrupted: Boolean) -> Unit = { }
) : AtomicCommand() {
    override val _isDone: Boolean
        get() = getDone.invoke()

    override fun execute() {
        _execute.invoke()
    }

    override fun start() {
        _start.invoke()
    }

    override fun done(interrupted: Boolean) {
        _done.invoke(interrupted)
    }
}