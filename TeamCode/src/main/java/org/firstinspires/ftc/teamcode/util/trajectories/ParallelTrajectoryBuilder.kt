package org.firstinspires.ftc.teamcode.util.trajectories

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import org.firstinspires.ftc.teamcode.commands.AtomicCommand

class ParallelTrajectoryBuilder(val builder: TrajectoryBuilder) {
    private val segmentLengths = mutableListOf<Double>()

    fun build(): ParallelTrajectory {
        return ParallelTrajectory(builder.build(), segmentLengths)
    }


}