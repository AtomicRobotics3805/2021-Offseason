package org.firstinspires.ftc.teamcode.util.commands

open class CustomCommand(
        private val getDone: () -> Boolean = { true },
        private val _run: () -> Unit = { },
        private val _start: () -> Unit = { },
        private val _done: () -> Unit = { }
) : AtomicCommand() {
    override val _isDone: Boolean
        get() = getDone.invoke()

    override fun run() {
        _run.invoke()
    }

    override fun start() {
        _start.invoke()
    }

    override fun done() {
        _done.invoke()
    }
}