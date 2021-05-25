package org.firstinspires.ftc.teamcode.util.commands

import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive

fun sequential(block: SequentialCommandGroup.() -> Unit): SequentialCommandGroup {
    return SequentialCommandGroup().apply(block)
}

fun parallel(block: ParallelCommandGroup.() -> Unit): ParallelCommandGroup {
    return ParallelCommandGroup().apply(block)
}

abstract class CommandGroup: AtomicCommand() {
    override val _isDone: Boolean
        get() = commands.isEmpty()

    val commands: MutableList<AtomicCommand> = mutableListOf()

    operator fun plusAssign(command: AtomicCommand) {
        commands += command
    }

    operator fun AtomicCommand.unaryPlus() = commands.add(this)

    override fun done(interrupted: Boolean) {
        for (command in commands) {
            command.done(interrupted)
        }
    }
}

class SequentialCommandGroup: CommandGroup() {
    override fun start() {
        if (commands.isNotEmpty())
            commands[0].start()
    }

    override fun execute() {
        if (commands.isNotEmpty()) {
            MecanumDrive.telemetry.addData("Command 0", commands[0])
            MecanumDrive.telemetry.addData("Command 0 _isDone", commands[0]._isDone)
            MecanumDrive.telemetry.addData("Command 0 isDone", commands[0].isDone)
            if (!commands[0].isDone)
                commands[0].execute()
            else {
                commands[0].done(false)
                commands.removeAt(0)
                if (commands.isNotEmpty())
                    commands[0].start()
            }
        }
    }
}

class ParallelCommandGroup: CommandGroup() {
    val toCancel: MutableList<AtomicCommand> = mutableListOf()

    override fun start() {
        for (command in commands) {
            command.start()
            command.isStarted = true
        }
    }

    override fun execute() {
        for (command in commands) {
            if (!command.isStarted) {
                command.start()
                command.isStarted = true
            }
            if (!command.isDone)
                command.execute()
            else {
                command.done(false)
                toCancel += commands
            }
        }
        for (command in toCancel) {
            commands -= command
        }
        toCancel.clear()
        MecanumDrive.telemetry.addData("This", this)
        MecanumDrive.telemetry.addData("Commands", commands)
        MecanumDrive.telemetry.addData("_isDone", _isDone)
        MecanumDrive.telemetry.addData("isDone", isDone)
        MecanumDrive.telemetry.addData("Commands Empty", commands.isEmpty())
        MecanumDrive.telemetry.update()
    }
}

