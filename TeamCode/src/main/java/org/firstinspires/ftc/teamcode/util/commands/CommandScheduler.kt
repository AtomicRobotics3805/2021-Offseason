package org.firstinspires.ftc.teamcode.util.commands

import org.firstinspires.ftc.teamcode.util.CommandGamepad


@Suppress("unused", "MemberVisibilityCanBePrivate")
object CommandScheduler {
    val commands = mutableListOf<AtomicCommand>()

    // these actions run whenever a command is inititialized, executed, interrupted, or finished
    val initActions = mutableListOf<(AtomicCommand) -> Unit>()
    val executeActions = mutableListOf<(AtomicCommand) -> Unit>()
    val interruptActions = mutableListOf<(AtomicCommand) -> Unit>()
    val finishActions = mutableListOf<(AtomicCommand) -> Unit>()

    // these gamepads have commands corresponding to certain buttons
    val gamepads = mutableListOf<CommandGamepad>()
    val subsystems = mutableListOf<Subsystem>()

    // exercise is healthy
    fun run() {
        updateGamepads()
        updateSubsystems()
        for (command in commands) {
            try {
                if (!command.isStarted)
                    initCommand(command)
            } catch(e: SubsystemBusyException) {
                commands -= command
                continue
            }
            if (command.isDone) {
                cancel(command, false)
                continue
            }
            command.execute()
        }
    }

    @Throws(SubsystemBusyException::class)
    fun initCommand(command: AtomicCommand) {
        for (requirement in command.requirements) {
            val conflicts = findCommands({ it.requirements.contains(requirement) })
            for (conflict in conflicts)
                if (!conflict.interruptable) {
                    doActions(interruptActions, conflict)
                    throw SubsystemBusyException(requirement)
                }
            for (conflict in conflicts)
                cancel(conflict, true)
            command.start()
            commands += command
        }
        doActions(initActions, command)
    }

    fun cancel(command: AtomicCommand, interrupted: Boolean = false) {
        command.done(interrupted)
        doActions(finishActions, command)
        commands -= command
    }

    fun cancelAll() {
        for (command in commands)
            cancel(command, true)
    }

    fun updateGamepads() {
        for (gamepad in gamepads)
            gamepad.update()
    }

    fun updateSubsystems() {
        for (subsystem in subsystems) {
            subsystem.periodic()
            if (findCommand({ it.requirements.contains(subsystem) }) != null)
                subsystem.inUsePeriodic()
        }
    }

    fun registerSubsystems(vararg subsystems: Subsystem) {
        for (subsystem in subsystems)
            this.subsystems += subsystem
    }

    fun registerGamepads(vararg gamepads: CommandGamepad) {
        for (gamepad in gamepads)
            this.gamepads += gamepad
    }

    fun doActions(actions: List<(AtomicCommand) -> Unit>, command: AtomicCommand) {
        for (action in actions)
            action.invoke(command)
    }

    fun findCommand(check: (AtomicCommand) -> Boolean, list: List<AtomicCommand> = commands) = findCommands(check, list).firstOrNull()

    fun findCommands(check: (AtomicCommand) -> Boolean, list: List<AtomicCommand> = commands): List<AtomicCommand> {
        val foundCommands = mutableListOf<AtomicCommand>()
        for (command in list) {
            if (check.invoke(command))
                foundCommands.add(command)
            if (command is CommandGroup) {
                val c = findCommand(check, command.commands)
                if (c != null) foundCommands.add(c)
            }
        }
        return foundCommands
    }
}