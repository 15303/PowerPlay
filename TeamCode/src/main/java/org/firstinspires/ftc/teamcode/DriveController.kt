package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.opmodes.angleDiff

//interface DriveController {
//    var targetAngle: Double
//
//    fun enableAngleCorrection(enabled: Boolean = true)
//    fun tick(drive: Double, turn: Double, strafe: Double)
//}

class DriveController(val robot: Robot) {
    var targetAngle: Double = 0.0
        set(value) {
            field = normalize(value)
        }
    var angleCorrection = false

    fun enableAngleCorrection(enabled: Boolean = true) {
        angleCorrection = enabled
    }

    fun tick(drive: Double, _turn: Double, strafe: Double) {
        val turn =
            if (angleCorrection) angleDiff(robot.getOrientation(), targetAngle) * 0.01
            else _turn

        robot.power(
            drive+turn+strafe,
            drive-turn-strafe,
            drive+turn-strafe,
            drive-turn+strafe,
        )
    }

}

fun normalize(_angle: Double): Double {
    var angle = _angle % 360
    if (angle >= 180) angle -= 360
    if (angle < -180) angle += 360
    return angle
}