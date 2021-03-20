package org.firstinspires.ftc.teamcode.commands

fun sequential(block: SequentialCommandGroup.() -> Unit): SequentialCommandGroup {
    return SequentialCommandGroup().apply(block)
}

fun parallel(block: ParallelCommandGroup.() -> Unit): ParallelCommandGroup {
    return ParallelCommandGroup().apply(block)
}

abstract class CommandGroup: AtomicCommand() {
    override val _isDone: Boolean
        get() = commands.isEmpty()

    protected val commands: MutableList<AtomicCommand> = mutableListOf()

    operator fun AtomicCommand.unaryPlus() = commands.add(this)
}

class SequentialCommandGroup: CommandGroup() {
    override fun run() {
        if (!commands[0].isDone) {
            if (!commands[0].isStarted) {
                commands[0].start()
                commands[0].isStarted = true
            }
            commands[0].run()
        }
        else {
            commands[0].done()
            commands.removeAt(0)
        }
    }
}

class ParallelCommandGroup: CommandGroup() {
    override fun start() {
        for(command in commands) {
            command.start()
        }
    }

    override fun run() {
        for(command in commands) {
            if (!command.isDone) {
                command.run()
            }
            else {
                command.done()
                commands.remove(command)
            }
        }
    }
}

