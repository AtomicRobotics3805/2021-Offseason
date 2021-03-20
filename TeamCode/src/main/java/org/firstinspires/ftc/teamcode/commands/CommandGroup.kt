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
    override fun start() {
        if (commands.isNotEmpty())
            commands[0].start()
    }

    override fun run() {
        if (commands.isNotEmpty()) {
            if (!commands[0].isDone)
                commands[0].run()
            else {
                commands[0].done()
                commands.removeAt(0)
                if (commands.isNotEmpty())
                    commands[0].start()
            }
        }
    }
}

class ParallelCommandGroup: CommandGroup() {
    override fun start() {
        for(command in commands)
            command.start()
    }

    override fun run() {
        for(command in commands) {
            if (!command.isDone)
                command.run()
            else {
                command.done()
                commands.remove(command)
            }
        }
    }
}

