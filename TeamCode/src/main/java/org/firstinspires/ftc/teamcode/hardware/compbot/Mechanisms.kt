@file:Suppress("MemberVisibilityCanBePrivate", "unused")

package org.firstinspires.ftc.teamcode.hardware.compbot

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Singleton.opMode
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.delays.Delay
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.INTAKE_NAME
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.LEFT_INDEXER_NAME
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.RIGHT_INDEXER_NAME
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.RING_DELAY
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.SHOOTER_NAME
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.WOBBLE_ARM_NAME
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.WOBBLE_CLAW_NAME

object Mechanisms {
    lateinit var intake: Intake
    lateinit var shooter: Shooter
    lateinit var wobble: Wobble
}

class Intake {
    private val motor: DcMotorEx = opMode.hardwareMap.get(DcMotorEx::class.java, INTAKE_NAME)

    init {
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun start(): AtomicCommand {
        return CustomCommand(_start = {motor.power = MechanismConstants.INTAKE_SPEED})
    }

    fun stop(): AtomicCommand {
        return CustomCommand(_start = {motor.power = 0.0})
    }
}

class Shooter {
    private val shooterMotor: DcMotorEx = opMode.hardwareMap.get(DcMotorEx::class.java, SHOOTER_NAME)
    private val leftIndexerServo: Servo = opMode.hardwareMap.get(Servo::class.java, LEFT_INDEXER_NAME)
    private val rightIndexerServo: Servo = opMode.hardwareMap.get(Servo::class.java, RIGHT_INDEXER_NAME)


    init {
        shooterMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun startMotor(): AtomicCommand {
        return CustomCommand(_start = {shooterMotor.power = 1.0})
    }

    fun stopMotor(): AtomicCommand {
        return CustomCommand(_start = {shooterMotor.power = MechanismConstants.SHOOTER_SPEED})
    }

    fun shootRing(): AtomicCommand {
        return sequential {
            +extendIndexerServos()
            +Delay(RING_DELAY / 2)
            +retractIndexerServos()
        }
    }

    fun extendIndexerServos(): AtomicCommand {
        return CustomCommand(_start = {
            leftIndexerServo.position = 0.6
            rightIndexerServo.position = 0.575
        })
    }

    fun retractIndexerServos(): AtomicCommand {
        return CustomCommand(_start = {
            leftIndexerServo.position = 0.925
            rightIndexerServo.position = 0.25
        })
    }
}

class Wobble {
    private val arm: DcMotorEx = opMode.hardwareMap.get(DcMotorEx::class.java, WOBBLE_ARM_NAME)
    private val claw: CRServo = opMode.hardwareMap.get(CRServo::class.java, WOBBLE_CLAW_NAME)

    init {
        arm.mode = DcMotor.RunMode.RUN_TO_POSITION
    }

    fun grab(): AtomicCommand {
        return parallel {
            +closeClaw()
            +sequential {
                Delay(0.8)
                raiseArm()
            }
        }
    }

    fun drop(): AtomicCommand {
        return openClaw()
    }

    fun raiseArmHigh(): AtomicCommand {
        return CustomCommand(_start = {
            arm.targetPosition = -300
        }, getDone = {
            !arm.isBusy
        })
    }

    fun raiseArm(): AtomicCommand {
        return CustomCommand(_start = {
            arm.targetPosition = -800
        }, getDone = {
            !arm.isBusy
        })
    }

    fun lowerArm(): AtomicCommand {
        return CustomCommand(_start = {
            arm.targetPosition = -1050
        }, getDone = {
            !arm.isBusy
        })
    }

    fun closeClaw(): AtomicCommand {
        return TimedCustomCommand(_start = {
            claw.power = 1.0
        }, time = 0.8)
    }

    fun openClaw(): AtomicCommand {
        return TimedCustomCommand(_start = {
            claw.power = -1.0
        }, time = 0.8)
    }

    fun idleClaw(): AtomicCommand {
        return TimedCustomCommand(_start = {
            claw.power = 0.0
        }, time = 0.8)
    }
}