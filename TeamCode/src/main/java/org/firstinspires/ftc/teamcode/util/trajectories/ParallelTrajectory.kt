package org.firstinspires.ftc.teamcode.util.trajectories

import com.acmerobotics.roadrunner.trajectory.Trajectory

data class ParallelTrajectory(private val trajectory: Trajectory,
                              private val segmentLengths: MutableList<Double>)