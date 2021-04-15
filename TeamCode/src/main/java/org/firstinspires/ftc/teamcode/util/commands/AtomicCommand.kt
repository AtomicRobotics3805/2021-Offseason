package org.firstinspires.ftc.teamcode.util.commands

import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.isStopRequested

abstract class AtomicCommand {
    var isDone = false
        get() = field || _isDone
    open val _isDone = true
    open val interruptable = true
    var isStarted = false

    open val requirements = mutableSetOf<Subsystem>()

    // exercise is healthy
    fun run() {
        start()
        while (opMode.isStopRequested || isDone) {
            execute()
        }
        done(opMode.isStopRequested)
    }
    open fun execute() { }
    open fun start() { }
    open fun done(interrupted: Boolean) { }
}