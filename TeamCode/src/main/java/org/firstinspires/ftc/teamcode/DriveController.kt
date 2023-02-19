package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

//interface DriveController {
//    var targetAngle: Double
//
//    fun enableAngleCorrection(enabled: Boolean = true)
//    fun tick(drive: Double, turn: Double, strafe: Double)
//}

open class DriveController(val robot: Robot) {
    val currentAngle: Double
        get () {
            return robot.getOrientation()
        }
    var targetAngle: Double = 0.0
        set(value) {
            field = normalize(value)
        }
    var angleCorrection = false
    val currentPos: Int
        get() {
            return robot.motors.sumOf { it.currentPosition }
        }
    var targetPos: Int = 0

    val telemetry: Telemetry = robot.opMode.telemetry

    fun enableAngleCorrection(enabled: Boolean = true) {
        angleCorrection = enabled
    }

    open fun tick(drive: Double, _turn: Double, strafe: Double) {
        val turn =
            if (angleCorrection) angleDiff(currentAngle, targetAngle) * 0.04 //TODO pid
            else _turn

        robot.power(
            drive+turn+strafe,
            drive-turn-strafe,
            drive+turn-strafe,
            drive-turn+strafe,
        )
    }

    open fun driveToTargetPos(
        _drive: Double = 0.2,
        _turn: Double = 0.5,
        targetPos: Int = this.currentPos,
        targetAngle: Double = this.targetAngle,
        time: Long = 2000
    ) {
        this.targetAngle = targetAngle
        this.targetPos = targetPos
        val starttime = System.currentTimeMillis()
        val positions = ArrayDeque(List(10) { currentPos })

        var lastt = System.nanoTime()

        while (robot.opMode.opModeIsActive() && System.currentTimeMillis() < starttime+time) {
            positions.removeFirst()
            positions.addLast(currentPos)
            val vel = positions.last() - positions.first()
            val drive = ((targetPos - currentPos) / 1000.0).coerceIn(-_drive, _drive)

            val turn =
                if (angleCorrection) (angleDiff(currentAngle, targetAngle) * 0.04).coerceIn(-_turn, _turn)
                else 0.0

            //try to prevent slipping
            val l = (drive+turn).coerceIn(min(0.0, vel/2000.0 - 0.8), max(0.0, vel/2000.0 + 0.8))
            val r = (drive-turn).coerceIn(min(0.0, vel/2000.0 - 0.8), max(0.0, vel/2000.0 + 0.8))

            robot.power(l, r, l, r)

            telemetry.addData("currentPos", currentPos)
            telemetry.addData("targetPos", targetPos)
            telemetry.addData("vel", vel)
            telemetry.addData("l", l)
            telemetry.addData("r", r)
            telemetry.addData("error", targetPos - currentPos)
            telemetry.update()

            //consistent loop times of 10ms
            val sleep = lastt / 1000000 + 10 - System.nanoTime() / 1000000
            if (sleep > 0) {
                robot.opMode.sleep(sleep)
            }
            lastt = System.nanoTime()
        }
        robot.drive(0)
    }

    open fun turnToTarget(
        _drive: Double = 0.2,
        _turn: Double = 0.5,
        targetPos: Int = this.currentPos,
        getCurrentVal: () -> Double,
        targetVal: Double,
        time: Long = 500
    ) {
        this.targetAngle = targetAngle
        this.targetPos = targetPos
        val starttime = System.currentTimeMillis()
        val values = ArrayDeque(List(1) { getCurrentVal() })

        var lastt = System.nanoTime()

        while (robot.opMode.opModeIsActive() && System.currentTimeMillis() < starttime+time) {
            values.removeFirst()
            values.addLast(getCurrentVal())
            val drive = ((targetPos - currentPos) / 1000.0).coerceIn(-_drive, _drive)

            var turn =
                ((targetVal - values.average()) * -0.001).coerceIn(-_turn, _turn)
            if (abs(turn) in 0.03..0.2) turn = 0.2 * sign(turn)

            robot.power(
                drive + turn,
                drive - turn,
                drive + turn,
                drive - turn
            )

            telemetry.addData("currentPos", currentPos)
            telemetry.addData("targetPos", targetPos)
            telemetry.addData("error", targetPos - currentPos)
            telemetry.addData("average", values.average())
            telemetry.addData("getCurrentVal", getCurrentVal())
            telemetry.addData("targetVal", targetVal)
            telemetry.addData("turn", turn)
            telemetry.update()

            //consistent loop times of 10ms
            val sleep = lastt / 1000000 + 10 - System.nanoTime() / 1000000
            if (sleep > 0) {
                robot.opMode.sleep(sleep)
            }
            lastt = System.nanoTime()
        }
        robot.drive(0)
    }


    open fun driveToTarget(
        _drive: Double = 0.2,
        _turn: Double = 0.5,
        targetPos: Int = this.currentPos,
        getCurrentVal: () -> Double,
        targetVal: Double,
        time: Long = 500
    ) {
        this.targetAngle = targetAngle
        this.targetPos = targetPos
        val starttime = System.currentTimeMillis()
        val values = ArrayDeque(List(1) { getCurrentVal() })

        var lastt = System.nanoTime()

        while (robot.opMode.opModeIsActive() && System.currentTimeMillis() < starttime+time) {
            values.removeFirst()
            values.addLast(getCurrentVal())
            val drive = ((targetPos - currentPos) / 1000.0).coerceIn(-_drive, _drive)

            robot.power(drive, drive, drive, drive)

            telemetry.addData("currentPos", currentPos)
            telemetry.addData("targetPos", targetPos)
            telemetry.addData("error", targetPos - currentPos)
            telemetry.addData("average", values.average())
            telemetry.addData("getCurrentVal", getCurrentVal())
            telemetry.addData("targetVal", targetVal)
            telemetry.update()

            //consistent loop times of 10ms
            val sleep = lastt / 1000000 + 10 - System.nanoTime() / 1000000
            if (sleep > 0) {
                robot.opMode.sleep(sleep)
            }
            lastt = System.nanoTime()
        }
        robot.drive(0)
    }

    //slow and steady wins the race
    fun slowDrive(
        offset: Int,
        _turn: Double = 0.5,
        _drive: Double = 0.35
    ) {
        val target = currentPos + offset
        if (offset >= 0) {
//            robot.drive(0.3)
            while (robot.opMode.opModeIsActive() && currentPos < target) {
                val turn =
                    if (angleCorrection) (angleDiff(currentAngle, targetAngle) * 0.04).coerceIn(-_turn, _turn)
                    else 0.0
                robot.power(
                    _drive + turn,
                    _drive - turn,
                    _drive + turn,
                    _drive - turn
                )
            }
        } else {
//            robot.drive(-0.3)
            while (robot.opMode.opModeIsActive() && currentPos > target) {
                val drive = -_drive
                val turn =
                    if (angleCorrection) (angleDiff(currentAngle, targetAngle) * 0.04).coerceIn(-_turn, _turn)
                    else 0.0
                robot.power(
                    drive + turn,
                    drive - turn,
                    drive + turn,
                    drive - turn
                )
            }
        }
        //angle correct for like 200 more ms
        val starttime = System.currentTimeMillis()
        while (robot.opMode.opModeIsActive() && System.currentTimeMillis() < 200 + starttime) {
//            val turn =
//                if (angleCorrection) (angleDiff(currentAngle, targetAngle) * 0.04).coerceIn(-_turn, _turn)
//                else 0.0
//            val drive = ((targetPos - currentPos) / 1000.0).coerceIn(-_drive, _drive)

            val drive = 0.0
            val turn =
                if (angleCorrection) (angleDiff(currentAngle, targetAngle) * 0.04).coerceIn(-_turn, _turn)
                else 0.0

            robot.power(
                drive+turn,
                drive-turn,
                drive+turn,
                drive-turn
            )
        }
        robot.drive(0)
        robot.opMode.sleep(100)
    }
}

class PIDDriveController(robot: Robot): DriveController(robot) {
    override fun driveToTargetPos(
        _drive: Double,
        _turn: Double,
        targetPos: Int,
        targetAngle: Double,
        time: Long
    ) {
        this.targetAngle = targetAngle
        this.targetPos = targetPos

        val starttime = System.currentTimeMillis()

//        val pid: MiniPID = MiniPID(0.000200, 0.0000120, 0.000090)
        val pid: MiniPID = MiniPID(0.000400, 0.00001, 0.0)
        pid.setOutputLimits(-_drive, _drive)
//        pid.setOutputRampRate(0.08)
        var lastt = System.nanoTime()

        while (robot.opMode.opModeIsActive() && System.currentTimeMillis() < starttime+time) {
            val pidoutput = pid.getOutput(currentPos.toDouble(), targetPos.toDouble())
            val drive = pidoutput.coerceIn(-_drive, _drive)
            val turn =
                if (angleCorrection) (angleDiff(currentAngle, targetAngle) * 0.04).coerceIn(-_turn, _turn)
                else 0.0

            robot.power(
                drive+turn,
                drive-turn,
                drive+turn,
                drive-turn
            )

            telemetry.addData("currentPos", currentPos)
            telemetry.addData("targetPos", targetPos)
            telemetry.addData("pidoutput", pidoutput)
            telemetry.addData("error", targetPos - currentPos)
            telemetry.update()

            //consistent loop times of 10ms
            val sleep = lastt / 1000000 + 10 - System.nanoTime() / 1000000
            if (sleep > 0) {
                robot.opMode.sleep(sleep)
            }
            lastt = System.nanoTime()
        }
        robot.drive(0)
    }
}

fun normalize(_angle: Double): Double {
    var angle = _angle % 360
    if (angle >= 180) angle -= 360
    if (angle < -180) angle += 360
    return angle
}

fun angleDiff(a: Double, b: Double) : Double {
    var res = (a - b) % 360
    if (res >= 180) res -= 360
    else if (res < -180) res += 360
    return res
}