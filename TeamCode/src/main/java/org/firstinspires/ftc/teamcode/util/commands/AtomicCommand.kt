package org.firstinspires.ftc.teamcode.util.commands

import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.isStopRequested

@Suppress("PropertyName")
abstract class AtomicCommand {
    var isDone = false
        get() = field || _isDone
    open val _isDone = true
    open val interruptible = true
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