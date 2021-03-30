package org.firstinspires.ftc.teamcode.subsystems.mechanisms

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.util.commands.*
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay

@Suppress("Unused", "MemberVisibilityCanBePrivate")
class Wobble {
    private lateinit var arm: DcMotorEx
    private lateinit var claw: CRServo

    fun initialize() {
        arm = Constants.opMode.hardwareMap.get(DcMotorEx::class.java, MechanismConstants.WOBBLE_ARM_NAME)
        claw = Constants.opMode.hardwareMap.get(CRServo::class.java, MechanismConstants.WOBBLE_CLAW_NAME)

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

    fun raiseArmHigh(): AtomicCommand = CustomCommand(
            _start = {
                arm.targetPosition = -300
            },
            getDone = {
                !arm.isBusy
            })

    fun raiseArm(): AtomicCommand = CustomCommand(
            _start = {
                arm.targetPosition = -800
            },
            getDone = {
                !arm.isBusy
            })

    fun lowerArm(): AtomicCommand = CustomCommand(
            _start = {
                arm.targetPosition = -1050
            },
            getDone = {
                !arm.isBusy
            })

    fun closeClaw(): AtomicCommand = TimedCustomCommand(
            _start = {
                claw.power = 1.0
            }, time = 0.8)

    fun openClaw(): AtomicCommand = TimedCustomCommand(
            _start = {
                claw.power = -1.0
            }, time = 0.8)

    fun idleClaw(): AtomicCommand = TimedCustomCommand(
            _start = {
                claw.power = 0.0
            }, time = 0.8)
}