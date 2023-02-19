package org.firstinspires.ftc.teamcode.opmodes

import android.R.id
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.PolarizarPipeline
import org.firstinspires.ftc.teamcode.PolarizetPipeline
import org.opencv.core.Mat
import org.opencv.core.Point
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline


@TeleOp(name = "PolarizetTest")
class PolarizetTest: LinearOpMode() {
    override fun runOpMode() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )

        telemetry.addLine("getting webcam")
        telemetry.update()
        val webcamName: WebcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        telemetry.addLine("creating webcam")
        telemetry.update()
        val camera: OpenCvCamera =
            OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        telemetry.addLine("setting pipeline")
        telemetry.update()
        val pipeline = PolarizetPipeline()
        camera.setPipeline(pipeline)

        val s = "opening camera"
        telemetry.addLine(s)
        telemetry.update()
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT)
                telemetry.addLine("opened")
                telemetry.update()
            }

            override fun onError(errorCode: Int) {
                /*
       * This will be called if the camera could not be opened
       */
            }
        })
        while (!isStarted) {
            telemetry.addData("pole_x", pipeline.pole_x)
            telemetry.update()
        }
        waitForStart()

    }
}