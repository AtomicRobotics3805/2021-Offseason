package org.firstinspires.ftc.teamcode.util.trajectories

import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import org.firstinspires.ftc.teamcode.commands.AtomicCommand

class ParallelTrajectoryBuilder(val builder: TrajectoryBuilder) {
    private val parallelMarkers: MutableList<ParallelMarker> = mutableListOf()

    fun build(): ParallelTrajectory {
        return ParallelTrajectory(builder.build(), parallelMarkers)
    }



    fun addTemporalMarker(time: Double, command: AtomicCommand) {
        parallelMarkers.add(ParallelMarker(time, command))
    }
}