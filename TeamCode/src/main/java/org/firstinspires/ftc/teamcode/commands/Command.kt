package org.firstinspires.ftc.teamcode.commands

abstract class Command {
    abstract val isDone: Boolean
    var isStarted = false

    // exercise is healthy
    abstract fun run()
    open fun start() { }
    open fun done() { }
}