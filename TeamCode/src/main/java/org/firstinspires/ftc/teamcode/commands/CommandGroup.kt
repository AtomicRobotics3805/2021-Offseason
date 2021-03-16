package org.firstinspires.ftc.teamcode.commands

fun sequential(block: CommandGroup.() -> Unit) {
    CommandGroup(CommandGroup.Mode.SEQUENTIAL).apply(block)
}

fun parallel(block: CommandGroup.() -> Unit) {
    CommandGroup(CommandGroup.Mode.PARALLEL).apply(block)
}

class CommandGroup(private val type: Mode)  {
    enum class Mode {
        SEQUENTIAL,
        PARALLEL
    }

    private val commands: MutableList<Command> = mutableListOf()

    operator fun Command.unaryPlus() = commands.add(this)

    fun run() {
        when(type) {
            Mode.SEQUENTIAL ->
                for(command in commands) {
                    command.start()
                    while(!command.isDone) {
                        command.run()
                    }
                    command.done()
                }
            Mode.PARALLEL ->
                while(commands.isNotEmpty()) {
                    for(command in commands) {
                        if (!command.isStarted) {
                            command.start()
                            command.isStarted = true
                        }
                        command.run()
                        if (!command.isDone) {
                            command.done()
                            commands.remove(command)
                        }
                    }
                }
        }
    }
}

