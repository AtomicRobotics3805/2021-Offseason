package org.firstinspires.ftc.teamcode.util.trajectories

import org.firstinspires.ftc.teamcode.Singleton.Color.*
import org.firstinspires.ftc.teamcode.Singleton.color

val Double.a get() = if (color == BLUE) this else 360 - this
val Double.y get () = (if (color == BLUE) this else this * -1)
val Double.flip get () = (if (color == BLUE) this else {
    if(this - 180 < 0) this + 180
    this - 180
})