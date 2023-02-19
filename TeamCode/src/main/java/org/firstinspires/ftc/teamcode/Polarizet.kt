package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.*
import org.opencv.features2d.BFMatcher
import org.opencv.imgproc.Imgproc
import org.opencv.calib3d.Calib3d
import org.openftc.easyopencv.OpenCvPipeline

class PolarizetPipeline : OpenCvPipeline() {

    var pole_x = 0.0;

    operator fun Point.plus(a: Point) = Point(this.x+a.x, this.y+a.y)
    operator fun Point.plus(a: Double) = Point(this.x+a, this.y+a)
    operator fun Double.plus(a: Point) = a + this

//    infix fun Point.plus(a: Point) = Point(this.x + a.x, this.y + a.y)
    operator fun Point.minus(a: Point) = Point(this.x - a.x, this.y - a.y)
    operator fun Point.minus(a: Double) = Point(this.x - a, this.y - a)
    operator fun Double.minus(a: Point) = -a + this

    infix fun Point.dot(a: Point) = this.x * a.x + this.y * a.y

    operator fun Point.times(a: Point) = Point(this.x * a.x, this.y * a.y)
    operator fun Point.times(a: Double) = Point(this.x * a, this.y * a)
    operator fun Double.times(a: Point) = a * this

    operator fun Point.unaryMinus() = this * -1.0
    operator fun Point.div(a: Double) = Point(this.x / a, this.y / a)

    operator fun Point.invoke(x: Int, y: Int): Point {
        return Point(x.toDouble(), y.toDouble())
    }

    private fun length(p: Point): Double {
        return Math.sqrt(p dot p)
    }


    /*
    mata times matb
    mata * matb
     */
    infix operator fun Mat.times(a: Point): Point {
        return Point(
            this.get(0,0)[0] * a.x + this.get(0, 1)[0] * a.y + this.get(0,2)[0],
            this.get(1,0)[0] * a.x + this.get(1, 1)[0] * a.y + this.get(1,2)[0]
        )
    }

    private fun mat_from_pts(pts: List<Point>): Mat {
        val m = Mat(pts.size, 1, CvType.CV_32FC2)
        for(i in pts.indices) {
            m.put(i, 0, pts[i].x, pts[i].y)
        }
        return m
    }
    private fun mat_from_pts_1c(pts: List<Point>): Mat {
        val m = Mat(pts.size, 2, CvType.CV_32F)
        for(i in pts.indices) {
            m.put(i, 0, pts[i].x, pts[i].y)
        }
        return m
    }
    private fun mat_from_pts(vararg pts: Point): Mat {
        return mat_from_pts(pts.asList())
    }
    private fun pts_from_mat(m: Mat): Array<Point> {
        val pts = Array(m.rows()) {
            Point(m.get(it, 0)[0], m.get(it, 0)[1])
        }
        return pts
    }

    /*
    finds the intersection of two lines
     */
    private fun seg_intersect(a1: Point, a2: Point, b1: Point, b2: Point): Point {
        val da = a2 - a1
        val db = b2 - b1
        val dp = a1 - b1
        val dap = Point(-da.y, da.x)
        val denom = dap dot db
        val num = dap dot dp
        return (db * (num / denom)) + b1
    }

    override fun processFrame(input: Mat): Mat {
        try {

            println("eaj --- PROCESS FRAME " + System.currentTimeMillis())

            val hsv = input.clone()
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)
            val lower = S(10, 120, 50)
            val upper = S(25, 255, 255)
            val mask = Mat()
            Core.inRange(hsv, lower, upper, mask)
            // Imgproc.cvtColor(mask, input, Imgproc.COLOR_GRAY2RGB)

            val Y = input.rows().toDouble()
            val X = input.cols().toDouble()
            val xfac = X/1280
            val yfac = Y/720

            val new_pts = arrayListOf<Point>()
            val contours = mutableListOf<MatOfPoint>()
            val hierarchy = Mat()
            Imgproc.findContours(
                mask,
                contours,
                hierarchy,
                Imgproc.RETR_TREE,
                Imgproc.CHAIN_APPROX_SIMPLE
            )

            println("eaj contours" + contours.size)

            var maxLength = 0.0;
            var closestBase = Point(0.0,0.0);
            for (contour in contours) {

                // Filter out small contours (likely noise)
                val area = Imgproc.contourArea(contour)

                // Reject small contours
                if (area < 100*xfac*yfac) continue

                val contour_pts = contour.toArray()

                // Find bounding rotated rectangle of contour (useful for extent of pole)
                val box = Imgproc.minAreaRect(MatOfPoint2f(*contour_pts))
                val box_pts = Array<Point?>(4) { null }
                box.points(box_pts)
                box_pts.sortByDescending { it!!.y }
                val left = box_pts[0]!!
                val right = box_pts[1]!!

                // Find center of mass of hull
                val moments = Imgproc.moments(contour)
                val center = Point(
                    moments.m10 / moments.m00,
                    moments.m01 / moments.m00
                )

                // Find line of best fit of hull
                val line = Mat()
                Imgproc.fitLine(contour, line, Imgproc.DIST_L2, 0.0, 0.01, 0.01)
                val slope = Point(line.get(0,0)[0], line.get(1,0)[0])

                if(Math.abs(slope.y/slope.x) < 1) continue
                // Calculate base of hull as intersection between the line of best fit passing through
                // the center of mass and the bottom edge of the bounding box

                val base = seg_intersect(center, center + slope, left, right)

                val length = length(center - base)
                if(length > maxLength) {
                    maxLength = length
                    closestBase = base
                }

                Imgproc.line(input, base, center, BLUE, 2)

                //Imgproc.circle(input, base, 10, GREEN, 2)
                //Imgproc.circle(warped_img, warped_base, 10, GREEN, 2)

                Imgproc.line(input, left, right, BLUE, 2)

                println("eaj a pole" + base)
            }

            pole_x = (closestBase.x - X/2)/xfac

            println("eaj deviation " + pole_x)
            Imgproc.circle(input, closestBase, 20, BLUE, 2)
            return input

        } catch (e: Exception) {
            println("eaj exception " + e.toString().replace("\n", " \\n "))
            println("eaj stacktrace " + e.stackTraceToString().replace("\n", " \\n "))
            return input
        }
    }

    companion object {
        val BLUE = S(0, 0, 255)
        val GREEN = S(0, 255, 0)
        val CYAN = S(0, 255, 255)
        val RED = S(255, 0, 0)
        val PINK = S(255, 0, 255)
        val YELLOW = S(255, 255, 0)
        val WHITE = S(255, 255, 255)

        fun S(vararg nums: Number): Scalar {
            return Scalar(nums.map { it.toDouble() }.toDoubleArray())
        }


    }
}