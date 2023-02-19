package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveController
import org.firstinspires.ftc.teamcode.Robot
import kotlin.coroutines.suspendCoroutine

fun LinearOpMode.runAuto(right: Boolean) {
    val robot = Robot(this)

    while (!isStarted) {
        telemetry.addData("current id", robot.camera?.lastRecognition?.id)
        telemetry.addData("initialized", robot.camera?.initialized)
        telemetry.update()
        sleep(50)
    }

    robot.reset()
    robot.lifter.targetPosition = 0
    robot.lifter.mode = DcMotor.RunMode.RUN_TO_POSITION
    robot.lifter.power = 0.75

    robot.motors.forEach { it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER }
    robot.motors.forEach { it.mode = DcMotor.RunMode.RUN_USING_ENCODER }

    val controller = DriveController(robot)
    controller.enableAngleCorrection()

    val startangle = controller.currentAngle

    val id = robot.camera?.lastRecognition?.id ?: 1
    telemetry.addLine("using id $id")
    telemetry.update()

    //swap to pole pipeline
    robot.camera?.swapPipelines()

    //begin by grabbing
    robot.grabber.power = -0.5;
    sleep(1000)

    //drive over to the medium pole
    //get cone out of way
    controller.slowDrive(6000)
    controller.slowDrive(-1000)
    controller.driveToTargetPos(
        targetAngle = startangle + if (right) 45 else -45,
        time = 1200
    )

    if (robot.camera != null) {
        controller.turnToTarget(
            getCurrentVal = { robot.camera.pole_x },
            targetVal = 315.0
        )
    }

    controller.targetAngle = controller.currentAngle

    //lift arm
    robot.lifter.targetPosition = Robot.LIFTER_MEDIUM_POS
    sleep(1000)
    controller.slowDrive(800)
    //release cone

    if (robot.camera != null) {
        controller.driveToTarget(
            getCurrentVal = { robot.camera.pole_x },
            targetVal = -540.0
        )
    }
    robot.lifter.targetPosition -= 50
    sleep(500)
    robot.grabber.power = 0.75
    sleep(250)
    robot.grabber.power = 0.0

    //return, turn to 0 degrees
    robot.lifter.targetPosition = Robot.LIFTER_GROUND_POS
    controller.slowDrive(-850)
    controller.driveToTargetPos(
        targetAngle = startangle,
        time = 1000
    )

    //drive to 3rd tile
    controller.slowDrive(4900)
    controller.slowDrive(-800)
    controller.driveToTargetPos(
        targetAngle = startangle + if (right) -90 else 90,
        time = 1500
    )

    //try grabbing cone from stack
    robot.lifter.targetPosition = 500
    controller.slowDrive(4300)
    robot.grabber.power = -0.5
    sleep(500)
    robot.lifter.targetPosition = Robot.LIFTER_LOW_POS
    sleep(500)

    //drive over to the low pole
    controller.slowDrive(-2300)

    controller.driveToTargetPos(
        targetAngle = startangle + 180,
        time = 1500
    )

    if (robot.camera != null) {
        controller.turnToTarget(
            getCurrentVal = { robot.camera.pole_x },
            targetVal = 320.0
        )
    }
    controller.targetAngle = controller.currentAngle

    controller.slowDrive(530)

    //release cone
    robot.lifter.targetPosition -= 50
    sleep(500)
    robot.grabber.power = 0.5
    sleep(250)
    robot.grabber.power = 0.0

    //drift
    robot.lifter.targetPosition = Robot.LIFTER_GROUND_POS + 100
    controller.slowDrive(-600)

    controller.driveToTargetPos(
        targetAngle = startangle + if (right) -90 else 90,
        time = 1500
    )

    //drive to parking spot
    when (id) {
        1 -> {
            if (right) controller.slowDrive(-6000)
            else controller.slowDrive(2000)
        }
        2 -> {
            controller.slowDrive(-2000)
        }
        3 -> {
            if (right) controller.slowDrive(2000)
            else controller.slowDrive(-6000)

        }
    }

    telemetry.addLine("done")
    telemetry.update()
    sleep(1000)
}