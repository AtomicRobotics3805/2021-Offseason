package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand
import org.firstinspires.ftc.teamcode.util.commands.parallel
import org.firstinspires.ftc.teamcode.util.commands.sequential

@Suppress("Unused")
object AutoRoutines {
    private val startPose = Pose2d()

    val initRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Wobble.raiseArm
                +Wobble.closeClaw
                +Intake.start
            }
        }

    val lowRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Shooter.start
                +sequential {
                    +MecanumDrive.followTrajectory(TrajectoryFactory.startToLow)
                    +Wobble.openClaw
                    +MecanumDrive.followTrajectory(TrajectoryFactory.lowToPowershot)
                    +shootTowerRoutine
                    +Intake.stop
                    +Wobble.lowerArm
                    +MecanumDrive.followTrajectory(TrajectoryFactory.powershotToWobble)
                    +Shooter.stop
                    +Wobble.grab
                    +parallel {
                        +Wobble.raiseArm
                        +sequential {
                            +MecanumDrive.followTrajectory(TrajectoryFactory.wobbleToLow)
                            +Wobble.openClaw
                            +MecanumDrive.followTrajectory(TrajectoryFactory.lowToPark)
                        }
                    }
                }
            }
        }

    private val shootTowerRoutine: AtomicCommand
        get() = sequential {
            Shooter.shootRing
            Shooter.shootRing
            Shooter.shootRing
        }
}