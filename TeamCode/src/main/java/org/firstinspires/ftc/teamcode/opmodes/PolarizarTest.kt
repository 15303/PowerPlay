package org.firstinspires.ftc.teamcode.opmodes

import android.R.id
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvPipeline


@TeleOp(name = "PolarizarTest")
class PolarizarTest: LinearOpMode() {
    override fun runOpMode() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )

        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val camera: OpenCvCamera =
            OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        camera.setPipeline(object: OpenCvPipeline() {
            override fun processFrame(input: Mat): Mat {
                Imgproc.rectangle(
                    input,
                    Point(
                        (input.cols() / 4).toDouble(),
                        (input.rows() / 4).toDouble()
                    ),
                    Point(
                        (input.cols() * (3f / 4f)).toDouble(),
                        (input.rows() * (3f / 4f)).toDouble()
                    ),
                    Scalar(0.0, 255.0, 0.0), 4
                )
                return input
            }

        })

        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(800, 448)
            }

            override fun onError(errorCode: Int) {
                /*
       * This will be called if the camera could not be opened
       */
            }
        })
        waitForStart()
    }
}