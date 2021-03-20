package org.firstinspires.ftc.teamcode.commands

abstract class AtomicCommand {
    var isDone = false
        get() = field || _isDone
    open val _isDone = true
    var isStarted = false

    // exercise is healthy
    open fun run() { }
    open fun start() { }
    open fun done() { }
}