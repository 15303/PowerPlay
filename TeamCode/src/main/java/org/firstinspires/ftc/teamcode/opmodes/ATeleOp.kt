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
        robot.lifter.power = 0.8

        driveController = DriveController(robot)

        var abstarget = 0
        var lifterdrift = 0

        var targetAngle = 0.0
        var grabberOn = false
        var last = System.nanoTime()
        while (opModeIsActive()) {
            val dt = System.nanoTime() - last
            last = System.nanoTime()

            // drivetrain
            val drive = (-gamepad1.left_stick_y - gamepad1.right_stick_y).toDouble()/if (gamepad1.x) 1 else 2
            val turn = gamepad1.left_stick_x.toDouble()/(if (gamepad1.x) 1 else 2)
            val strafe = gamepad1.right_stick_x.toDouble()

            driveController.enableAngleCorrection(gamepad1.left_bumper)
            if (gamepad1.left_bumper) {
                driveController.targetAngle = targetAngle
            } else {
                targetAngle = robot.getOrientation()
            }
            driveController.tick(drive, turn, strafe)
            telemetry.addData("drive", drive)
            telemetry.addData("turn", turn)
            telemetry.addData("strafe", strafe)
            telemetry.addData("targetAngle", targetAngle)


            // lifter
//            if ((robot.lifter.currentPosition > 10 || want > 0) &&
//                (robot.lifter.currentPosition < 4440 || want < 0)) {
//                robot.lifter.power = want
//            } else {
//                robot.lifter.power = 0.0
//            }

            if (gamepad1.dpad_up || gamepad2.dpad_up) lifterdrift+=3
            if (gamepad1.dpad_down || gamepad2.dpad_down) lifterdrift-=3
//            if (gamepad1.dpad_right || gamepad2.dpad_right) {
//                robot.lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
//                robot.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
//            }

            abstarget = if (gamepad2.a) Robot.LIFTER_GROUND_POS
                else if (gamepad2.b && !gamepad2.start) Robot.LIFTER_LOW_POS
                else if (gamepad2.x) Robot.LIFTER_MEDIUM_POS
                else if (gamepad2.y) Robot.LIFTER_HIGH_POS
                else abstarget

            val wish = (-gamepad2.left_stick_y.toDouble() - gamepad2.right_stick_y) * dt / 800000
            if (wish != 0.0) abstarget += wish.toInt()

            abstarget = abstarget.coerceIn(0, 4440)
            robot.lifter.targetPosition = abstarget + lifterdrift

            telemetry.addData("lifter", robot.lifter.currentPosition)
            telemetry.addData("lifterdrift", lifterdrift)

            telemetry.addData("wish", wish)
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