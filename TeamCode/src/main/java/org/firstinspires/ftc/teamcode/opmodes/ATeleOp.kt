package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveController
import org.firstinspires.ftc.teamcode.Robot

const val GRABBER_ON_POWER = -0.5

@TeleOp(name = "ATeleOp")
class ATeleOp : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var driveController: DriveController
    override fun runOpMode() {
        robot = Robot(this)
//        robot.enableEncoders()

        waitForStart()

        robot.reset()
        robot.lifter.targetPosition = 0
        robot.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.lifter.power = 0.75

        driveController = DriveController(robot)

        var targetAngle = 0.0
        var grabberOn = false
        var last = System.nanoTime()
        while (opModeIsActive()) {
            val dt = System.nanoTime() - last
            last = System.nanoTime()

            // drivetrain
            val drive = (-gamepad1.left_stick_y).toDouble()/if (gamepad1.x) 1 else 2
            val turn = gamepad1.left_stick_x.toDouble()/(if (gamepad1.x) 1 else 2)
            val strafe = gamepad1.right_stick_x.toDouble()

            driveController.enableAngleCorrection(gamepad1.left_bumper)
            if (gamepad1.left_bumper) {
                driveController.targetAngle = targetAngle
            } else {
                targetAngle = robot.getOrientation()
            }
            driveController.tick(drive, turn, strafe)

            telemetry.addData("targetAngle", targetAngle)


            // lifter
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

            val wish = (-gamepad2.left_stick_y.toDouble() - gamepad2.right_stick_y) * dt / 1000000
            if (wish != 0.0) robot.lifter.targetPosition += wish.toInt()

            robot.lifter.targetPosition = robot.lifter.targetPosition.coerceIn(0, 4440)

            if (robot.lifter.targetPosition < 25 && robot.lifter.currentPosition < 75) {
                robot.lifter.power = 0.0
            } else if (robot.lifter.power == 0.0) {
                robot.lifter.power = 0.75
            }
            telemetry.addData("lifter", robot.lifter.currentPosition)

            telemetry.addData("want", wish)
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