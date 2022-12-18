package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Robot
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor

@TeleOp(name = "AnotherTeleOp")
class AnotherTeleOp : LinearOpMode() {
    lateinit var robot: Robot
    override fun runOpMode() {
        robot = Robot(this)
//        robot.enableEncoders()

        waitForStart()

        robot.reset()
//        robot.lifter.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.lifter.targetPosition = 0
        robot.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.lifter.power = 0.75

        var grabberOn = false
        var last = System.nanoTime()
        while (opModeIsActive()) {
            val dt = System.nanoTime() - last
            last = System.nanoTime()

            val drive = (-gamepad1.left_stick_y).toDouble()/if (gamepad1.x) 1 else 2
            var turn = gamepad1.left_stick_x.toDouble()/(if (gamepad1.x) 1 else 2)

            val theta = robot.getOrientation()
            val phi = (if (gamepad1.left_bumper) 1 else 0)*floor(theta/90)*90
            + (if (gamepad1.right_bumper) 1 else 0)*ceil(theta/90)*90
            val psi = angleDiff(theta, phi)
            turn += psi

            val strafe = gamepad1.right_stick_x.toDouble()
            telemetry.addData("drive", drive)
            telemetry.addData("turn", turn)
            telemetry.addData("strafe", strafe)
            telemetry.addData("angle", robot.getOrientation())
            telemetry.addData("lifter", robot.lifter.currentPosition)
            robot.power(
                drive+turn+strafe,
                drive-turn-strafe,
                drive+turn-strafe,
                drive-turn+strafe,
            )

            //0 to 4440

//            if ((robot.lifter.currentPosition > 10 || want > 0) &&
//                (robot.lifter.currentPosition < 4440 || want < 0)) {
//                robot.lifter.power = want
//            } else {
//                robot.lifter.power = 0.0
//            }
            if (gamepad2.a) robot.lifter.targetPosition = 0
            else if (gamepad2.b && !gamepad2.start) robot.lifter.targetPosition = 1680
            else if (gamepad2.x) robot.lifter.targetPosition = 2930
            else if (gamepad2.y) robot.lifter.targetPosition = 4100

            val want = ((-gamepad2.left_stick_y - gamepad2.right_stick_y) *
                    dt / 1000000).toInt()
            if (want != 0) robot.lifter.targetPosition += want

            robot.lifter.targetPosition = robot.lifter.targetPosition.coerceIn(0, 4440)

            if (robot.lifter.targetPosition < 25 && robot.lifter.currentPosition < 75) {
                robot.lifter.power = 0.0
            } else if (robot.lifter.power == 0.0) {
                robot.lifter.power = 0.75
            }
            //1680
            //2930
            //4000

            telemetry.addData("want", want)
            telemetry.addData("liftertarget", robot.lifter.targetPosition)

//            if (gamepad2.a) {
//                grabberOn = true
//            } else if (gamepad2.b) {
//                grabberOn = false
//            }
            robot.grabber.power = (gamepad2.right_trigger - gamepad2.left_trigger).toDouble()*2/3


            telemetry.update()
        }


    }
}

fun angleDiff(a: Double, b: Double) : Double {
    var res = (a - b) % 360
    if (res >= 180) res -= 360
    else if (res < -180) res += 360
    return res
}
