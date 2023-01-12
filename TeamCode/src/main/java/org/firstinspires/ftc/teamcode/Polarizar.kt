package org.firstinspires.ftc.teamcode

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class PolarizarPipeline: OpenCvPipeline() {
    override fun processFrame(input: Mat): Mat {
        val hsv = input.clone()
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_BGR2HSV)
        val lower = S(10, 120, 50)
        val upper = S(25,255,255)
        val mask = Mat()
        Core.inRange(hsv, lower, upper, mask)

        val src_plane = Mat(4,2, CvType.CV_32F)
        //sets 1,3 to 0 i think
        src_plane.at(Float::class.java, 1, 3).v = 0f
        //todo
        src_plane.put()
        Imgproc.getPerspectiveTransform()

        Core.inRange()
        TODO("Not yet implemented")
    }

    companion object {
        val RED =   S(0,0,255)
        val GREEN = S(0,255,0)
        val YELLOW =    S(0,255,255)
        val BLUE =  S(255,0,0)
        val PINK =  S(255,0,255)
        val CYAN =  S(255,255,0)
        val WHITE = S(255,255,255)

        fun S(vararg nums: Number): Scalar {
            return Scalar(nums.map { it.toDouble() }.toDoubleArray())
        }


    }
}