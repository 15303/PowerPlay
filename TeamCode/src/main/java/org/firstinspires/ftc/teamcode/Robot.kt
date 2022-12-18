package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.Action.*
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvPipeline
import org.openftc.easyopencv.OpenCvWebcam

class Robot(val opMode: LinearOpMode) {
    val fl: DcMotor = getMotor("front_left")
    val fr: DcMotor = getMotor("front_right")
    val bl: DcMotor = getMotor("back_left")
    val br: DcMotor = getMotor("back_right")

    val motors = listOf(fl, fr, bl, br)

    val lifter: DcMotor = getMotor("lifter")

    val grabber: CRServo = getServo("grabber")


    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".
    val imu: BNO055IMU = opMode.hardwareMap.get(BNO055IMU::class.java, "imu")


    init {
        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE

        lifter.direction = DcMotorSimple.Direction.REVERSE


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.calibrationDataFile =
            "BNO055IMUCalibration.json" // see the calibration sample opmode
        parameters.loggingEnabled = true
        parameters.loggingTag = "IMU"
        parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()

        imu.initialize(parameters)
    }

    private fun getMotor(name: String): DcMotor {
        return opMode.hardwareMap.get(name) as DcMotor
    }

    private fun getServo(name: String): CRServo {
        return opMode.hardwareMap.get(name) as CRServo
    }

    fun getOrientation(): Double {
        return imu.angularOrientation.firstAngle.toDouble()
    }

    fun drive(power: Double) = power(power, power, power, power)

    fun power(fl: Double, fr: Double, bl: Double, br: Double) {
        this.fl.power = fl
        this.fr.power = fr
        this.bl.power = bl
        this.br.power = br
    }

    infix fun go(action: Action): Robot {
        when (action) {
            forwards -> {
                drive(0.5)
            }
            backwards -> {
                drive(-0.5)
            }
            right -> {
                power(0.5, -0.5, 0.5, -0.5)
            }
            left -> {
                power(0.5, -0.5, 0.5, -0.5)
            }
        }
        return this
    }

    infix fun time(time: Long) {
        opMode.sleep(time)
        drive(0.0)
    }

    fun enableEncoders() {
//        for (m in motors) {
//            m.mode = DcMotor.RunMode.RUN_USING_ENCODER
//        }
    }

    fun reset() {
        lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }
}

enum class Action {
    forwards, backwards, left, right
}

class Camera(robot: Robot) {
    val webcam: OpenCvWebcam

    init {
        val cameraMonitorViewId: Int = robot.opMode.hardwareMap.appContext.resources
            .getIdentifier("cameraMonitorViewId", "id", robot.opMode.hardwareMap.appContext.getPackageName())

        val camera = robot.opMode.hardwareMap.get("Webcam 1") as WebcamName
        webcam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId)
        webcam.setPipeline(ConePipeline())
    }

    class ConePipeline: OpenCvPipeline() {
        override fun processFrame(input: Mat): Mat {

            return input
        }

    }

    companion object {
        private const val VUFORIA_KEY =
            "AV1AlC//////AAABmZ8844uwbEHrg1LUsHVFKxlStW4C7oPMwyIXaVB2lFgVrXI7AcN37g06/oHM+7Smo0UtpZXtGANu2IWFTeqOdHO83zy8s3nw7ZfZ60OUz9L230sZ0liJbP8aeIKa0a0ibeL+mH4zJTOHU/3rdfcv8PbufYdeMh1ImaoFXTXQkMqiELuxK32/kvH/sRyvMg5JmoQDxKgSgNhN/Vle754F6hCOVk1alZE7H5gXHifhPtL0Gf+AhkrfsbKi+zeZ3gRoGLzX54Qq8EUmOhlm5+ZMbdYkx1F4u8FoLczDK+Qt4J23kEqkbCA5HyDJJsmyA30/fEIYDEepO9f86U96LfOCIFt8Q3vFCWSq4IJZphMlVOF7"
    }

}