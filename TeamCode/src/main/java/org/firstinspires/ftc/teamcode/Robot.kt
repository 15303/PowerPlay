package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvWebcam

class Robot(val opMode: LinearOpMode) {
    val fl: DcMotor = getMotor("front_left")
    val fr: DcMotor = getMotor("front_right")
    val bl: DcMotor = getMotor("back_left")
    val br: DcMotor = getMotor("back_right")

    val motors = listOf(fl, fr, bl, br)

    val lifter: DcMotor = getMotor("lifter")

    val grabber: CRServo = getServo("grabber")

    val imu: BNO055IMU = opMode.hardwareMap.get(BNO055IMU::class.java, "imu")

    val camera: RobotCamera?

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

        camera = try {
            RobotCamera(this)
        } catch (e: Exception) {
            opMode.telemetry.addLine("Error enabling camera: $e")
            opMode.telemetry.update()
            null
        }
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

    fun drive(power: Number) = power(power, power, power, power)

    fun power(fl: Number, fr: Number, bl: Number, br: Number) {
        this.fl.power = fl.toDouble()
        this.fr.power = fr.toDouble()
        this.bl.power = bl.toDouble()
        this.br.power = br.toDouble()
    }

    fun reset() {
        lifter.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lifter.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        for (motor in motors) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    companion object {
        const val LIFTER_GROUND_POS = 0
        const val LIFTER_LOW_POS = 1680
        const val LIFTER_MEDIUM_POS = 2930
        const val LIFTER_HIGH_POS = 4100
    }
}

class RobotCamera(robot: Robot) {
    val webcam: OpenCvWebcam
    val pipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)
    var lastRecognition: AprilTagDetection? = null
        get() {
            field = pipeline.latestDetections.maxByOrNull { it.decisionMargin } ?: field
            return field
        }

    init {
        val cameraMonitorViewId: Int = robot.opMode.hardwareMap.appContext.resources
            .getIdentifier("cameraMonitorViewId", "id", robot.opMode.hardwareMap.appContext.getPackageName())

        val camera = robot.opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1")
        webcam = OpenCvCameraFactory.getInstance().createWebcam(camera, cameraMonitorViewId)
        webcam.setPipeline(pipeline)

        webcam.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(800, 448)
            }

            override fun onError(errorCode: Int) {}
        })
    }

    companion object {
        private const val VUFORIA_KEY =
            "AV1AlC//////AAABmZ8844uwbEHrg1LUsHVFKxlStW4C7oPMwyIXaVB2lFgVrXI7AcN37g06/oHM+7Smo0UtpZXtGANu2IWFTeqOdHO83zy8s3nw7ZfZ60OUz9L230sZ0liJbP8aeIKa0a0ibeL+mH4zJTOHU/3rdfcv8PbufYdeMh1ImaoFXTXQkMqiELuxK32/kvH/sRyvMg5JmoQDxKgSgNhN/Vle754F6hCOVk1alZE7H5gXHifhPtL0Gf+AhkrfsbKi+zeZ3gRoGLzX54Qq8EUmOhlm5+ZMbdYkx1F4u8FoLczDK+Qt4J23kEqkbCA5HyDJJsmyA30/fEIYDEepO9f86U96LfOCIFt8Q3vFCWSq4IJZphMlVOF7"

        const val tagsize = 0.166
        const val fx = 578.272
        const val fy = 578.272
        const val cx = 402.145
        const val cy = 221.506
    }

}