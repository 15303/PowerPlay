package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot
import kotlin.math.sign

@TeleOp(name = "ATeleOp1P")
@Disabled
class ATeleOp1P : LinearOpMode() {
    lateinit var robot: Robot
    override fun runOpMode() {
        robot = Robot(this)
//        robot.enableEncoders()

        waitForStart()

        var grabberOn = false
        while (opModeIsActive()) {
            val drive = (-gamepad1.left_stick_y).toDouble()/if (gamepad1.x) 1 else 2
            val turn = gamepad1.left_stick_x.toDouble()/(if (gamepad1.x) 1 else 2)
            val strafe = gamepad1.right_stick_x.toDouble()
            telemetry.addData("drive", drive)
            telemetry.addData("turn", turn)
            telemetry.addData("strafe", strafe)
            telemetry.update()
            robot.power(
                drive+turn+strafe,
                drive-turn-strafe,
                drive+turn-strafe,
                drive-turn+strafe,
            )

            robot.lifter.power = gamepad2.left_stick_y.toDouble()

            if (gamepad2.a) {
                grabberOn = true
            } else if (gamepad2.b) {
                grabberOn = false
            }
            robot.grabber.power =
                if (gamepad2.a) GRABBER_ON_POWER
                else if (gamepad2.b) -GRABBER_ON_POWER.sign
                else if (grabberOn) GRABBER_ON_POWER
                else 0.0

        }
    }
}