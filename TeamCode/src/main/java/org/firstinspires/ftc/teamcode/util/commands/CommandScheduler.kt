package org.firstinspires.ftc.teamcode.util.commands

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.CommandGamepad
import org.firstinspires.ftc.teamcode.util.commands.subsystems.Subsystem
import org.firstinspires.ftc.teamcode.util.commands.subsystems.SubsystemBusyException


@Suppress("unused", "MemberVisibilityCanBePrivate")
object CommandScheduler {
    val commands = mutableListOf<AtomicCommand>()
    val commandsToSchedule = mutableListOf<AtomicCommand>()
    val commandsToCancel = mutableMapOf<AtomicCommand, Boolean>()

    // these actions run whenever a command is initialized, executed, interrupted, or finished
    val initActions = mutableListOf<(AtomicCommand) -> Unit>()
    val executeActions = mutableListOf<(AtomicCommand) -> Unit>()
    val interruptActions = mutableListOf<(AtomicCommand) -> Unit>()
    val finishActions = mutableListOf<(AtomicCommand) -> Unit>()

    // these gamepads have commands corresponding to certain buttons
    val gamepads = mutableListOf<CommandGamepad>()
    val subsystems = mutableListOf<Subsystem>()

    val timer = ElapsedTime()

    // exercise is healthy
    fun run() {
        updateGamepads()
        updateSubsystems()
        scheduleCommands()
        cancelCommands()
        for (command in commands) {
            command.execute()
            if (command.isDone) {
                commandsToCancel += Pair(command, false)
            }
        }
    }

    fun scheduleCommands() {
        for (command in commandsToSchedule)
            try {
                initCommand(command)
            } catch(e: SubsystemBusyException) { }
        commandsToSchedule.clear()
    }

    fun cancelCommands() {
        for (pair in commandsToCancel)
            cancel(pair.key, pair.value)
        commandsToCancel.clear()
    }

    @Throws(SubsystemBusyException::class)
    fun initCommand(command: AtomicCommand) {
        for (requirement in command.requirements) {
            val conflicts = findCommands({ it.requirements.contains(requirement) }).toMutableList()
            if (conflicts.contains(command)) {
                conflicts -= command
            }
            for (conflict in conflicts)
                if (!conflict.interruptible) {
                    doActions(interruptActions, conflict)
                    throw SubsystemBusyException(requirement)
                }
            for (conflict in conflicts)
                commandsToCancel += Pair(command, true)
        }
        command.start()
        commands += command
        doActions(initActions, command)
    }

    fun cancel(command: AtomicCommand, interrupted: Boolean = false) {
        command.done(interrupted)
        doActions(finishActions, command)
        commands -= command
    }

    fun cancelAll() {
        for (command in commands)
            commandsToCancel += Pair(command, true)
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