package org.firstinspires.ftc.teamcode.util.commands.delays

import com.acmerobotics.roadrunner.profile.MotionProfile
import com.acmerobotics.roadrunner.util.epsilonEquals
import org.firstinspires.ftc.teamcode.Constants.drive
import org.firstinspires.ftc.teamcode.util.commands.AtomicCommand

@Suppress("Unused")
class DisplacementDelay(private val displacement: Double): AtomicCommand() {
    constructor(segmentNumber: Int) : this(drive.trajectory?.segmentLengths?.get(segmentNumber) ?: 0.0)

    override val _isDone: Boolean
        get() = displacementToTime(drive.follower.trajectory.profile, displacement) > drive.follower.elapsedTime()
}

private fun displacementToTime(profile: MotionProfile, s: Double): Double {
    var tLo = 0.0
    var tHi = profile.duration()
    while (!(tLo epsilonEquals tHi)) {
        val tMid = 0.5 * (tLo + tHi)
        if (profile[tMid].x > s) {
            tHi = tMid
        } else {
            tLo = tMid
        }
    }
    return 0.5 * (tLo + tHi)
}