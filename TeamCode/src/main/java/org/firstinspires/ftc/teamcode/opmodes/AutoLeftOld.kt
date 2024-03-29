package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveController
import org.firstinspires.ftc.teamcode.Robot

@Autonomous(name = "AutoLeftOld")
@Disabled
class AutoLeftOld : LinearOpMode() {
    lateinit var robot: Robot
    lateinit var controller: DriveController

    override fun runOpMode() {
        robot = Robot(this)

        while (!isStarted) {
            telemetry.addData("current id", robot.camera?.lastRecognition?.id)
            telemetry.update()
            sleep(50)
        }

        robot.reset()
        robot.lifter.targetPosition = 0
        robot.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
        robot.lifter.power = 0.75
        robot.motors.forEach { it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
        robot.motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }
        robot.motors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        controller = DriveController(robot)
        controller.enableAngleCorrection()

        val id = robot.camera?.lastRecognition?.id ?: 1
        telemetry.addLine("using id $id")
        telemetry.update()

        //begin by grabbing
        robot.grabber.power = -0.5;
        sleep(2000)

        //drive over to the medium pole
        controller.slowDrive(5000)
//        controller.driveToTargetPos(
//            _drive = 0.5,
//            targetPos = controller.currentPos + 5500,
//            time = 3000)
        controller.driveToTargetPos(
            targetAngle = controller.currentAngle - 45,
            time = 1500
        )
        //lift arm
        robot.lifter.targetPosition = Robot.LIFTER_MEDIUM_POS
        sleep(1000)
        controller.slowDrive(700)
//        controller.driveToTargetPos(
//            _drive = 0.3,
//            targetPos = controller.currentPos + 1150
//        )
        //release cone
        robot.lifter.targetPosition -= 50
        sleep(500)
        robot.grabber.power = 0.5
        sleep(250)
        robot.grabber.power = 0.0

        //return, turn to 0 degrees
        robot.lifter.targetPosition = Robot.LIFTER_GROUND_POS
        controller.slowDrive(-1700)
//        controller.driveToTargetPos(
//            _drive = 0.3,
//            targetPos = controller.currentPos - 1500
//        )
        controller.driveToTargetPos(
            targetAngle = controller.currentAngle + 45
        )

        //drive to 3rd tile
        controller.slowDrive(3700)
        controller.driveToTargetPos(
            targetAngle = controller.currentAngle + 90,
            time = 1500
        )

        //drive to parking spot
        when (id) {
            1 -> {
                controller.slowDrive(4200)

//                controller.driveToTargetPos(
//                    _drive = 0.5,
//                    targetPos = controller.currentPos - 4000,
//                    time = 3000)
            }
            2 -> {
                //do nothing, already in 2
            }
            3 -> {
                controller.slowDrive(-4200)

//                controller.driveToTargetPos(
//                    _drive = 0.5,
//                    targetPos = controller.currentPos + 4000,
//                    time = 3000)
            }
        }

        telemetry.addLine("done")
        telemetry.update()
        sleep(1000)
    }
}