package org.firstinspires.ftc.teamcode

import org.opencv.core.*
import org.opencv.features2d.BFMatcher
import org.opencv.imgproc.Imgproc
import org.opencv.calib3d.Calib3d
import org.openftc.easyopencv.OpenCvPipeline

class PolarizarPipeline : OpenCvPipeline() {
    val START_POS = Point(1.35, 1.8)
    val CORNER_POS = Point(3.0, 3.0)
    val GRID_SIZE = 250.0
    val TRUE_PTS = listOf(
        Point(-1.0, -2.0), Point(+1.0, -2.0),
        Point(-2.0, -1.0), Point(-1.0, -1.0), Point(0.0, -1.0), Point(+1.0, -1.0), Point(+2.0, -1.0),
        Point(-1.0, 0.0), Point(+1.0, 0.0),
        Point(-2.0, +1.0), Point(-1.0, +1.0), Point(0.0, +1.0), Point(+1.0, +1.0), Point(+2.0, +1.0),
        Point(-1.0, +2.0), Point(+1.0, +2.0)

    )
    val MAP_SIZE = 700.0

    val matcher = BFMatcher(Core.NORM_L2, true)

    var old_pos = START_POS

    operator fun Point.plus(a: Point) = Point(this.x+a.x, this.y+a.y)
    operator fun Point.plus(a: Double) = Point(this.x+a, this.y+a)

//    infix fun Point.plus(a: Point) = Point(this.x + a.x, this.y + a.y)
    operator fun Point.minus(a: Point) = Point(this.x - a.x, this.y - a.y)
    operator fun Point.minus(a: Double) = Point(this.x - a, this.y - a)

    infix fun Point.dot(a: Point) = this.x * a.x + this.y * a.y
    operator fun Point.times(a: Double) = Point(this.x * a, this.y * a)
    operator fun Point.div(a: Double) = Point(this.x / a, this.y / a)

    private fun length(p: Point): Double {
        return Math.sqrt(p dot p)
    }
    private fun warp(p: Point, X: Double, Yf: Double): Point {
        return (p - old_pos)*GRID_SIZE + Point(X/2.0, Yf)
    }
    private fun unwarp(p: Point, X: Double, Yf: Double): Point {
        return (Point(X / 2.0, Yf) - p)/GRID_SIZE
    }
    private fun map(p: Point, X: Double, Yf: Double): Point {
        return p * MAP_SIZE/7.0 + Point(X/2.0, Yf/2.0)
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
            val w = 1500.0
            val Yf = Y * 1.4
            val src_plane = mat_from_pts(
                Point(0.0, 0.0),
                Point(X, 0.0),
                Point(X + w, Y),
                Point(- w, Y),
            )
            val project_plane = mat_from_pts(
                Point(0.0, 0.0),
                Point(X, 0.0),
                Point(X, Yf),
                Point(0.0, Yf)
            )
            println("eaj src_plane  " + src_plane.dump().replace("\n",";"))
            println("eaj proj_plane  " + project_plane.dump().replace("\n",";"))

            val project_mat = Imgproc.getPerspectiveTransform(src_plane, project_plane)
            println("eaj proj  " + project_mat.dump().replace("\n",";"))

            val warped_img = Mat()
            Imgproc.warpPerspective(input, warped_img, project_mat, Size(X, Yf))

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

//        hsv.release()
////        input.release()
//        hierarchy.release() //todo remove?\

//        return warped_img

            println("eaj contours" + contours.size)
            for (contour in contours) {

                // Filter out small contours (likely noise)
                val area = Imgproc.contourArea(contour)

                // Reject small contours
                if (area < 100) continue

                println("eaj a valid contour ")

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
                val slope = Point(line.get(0, 0)[0], line.get(1, 0)[0])

                // Calculate base of hull as intersection between the line of best fit passing through
                // the center of mass and the bottom edge of the bounding box

                val base = seg_intersect(center, center + slope, left, right)

                println("eaj base" + base)
                // Store attributes of pole
                val pole = mat_from_pts(base, left, right, center)
                println("eaj pole " + pole.dump().replace("\n",";"))

                // Perspective warp pole features
                val warped_pole = Mat(4, 1, CvType.CV_32FC2)
                Core.perspectiveTransform(pole, warped_pole, project_mat)
                println("eaj warped pole "+warped_pole.dump().replace("\n",";"))
                val (warped_base, warped_left, warped_right, warped_center) = pts_from_mat(warped_pole)
                println("eaj deconstructed warped pole " + warped_base)

                // Camera is at center bottom of input view
                val rel_pos = unwarp(warped_base, X, Yf)
                val new_pt = old_pos - rel_pos

                new_pts.add(new_pt)

                Imgproc.line(input, base, center, BLUE, 2)
                Imgproc.line(warped_img, warped_base, warped_center, BLUE, 2)

                //Imgproc.circle(input, base, 10, GREEN, 2)
                //Imgproc.circle(warped_img, warped_base, 10, GREEN, 2)

                Imgproc.line(input, left, right, BLUE, 2)
                Imgproc.line(warped_img, warped_left, warped_right, BLUE, 2)

                println("eaj a pole" + warped_base)
            }
            println("eaj detected " + new_pts.size)

            val matched_true_pts = arrayListOf<Point>()
            val matched_new_pts = arrayListOf<Point>()

            if (new_pts.size >= 2) {
                // Match up true and observed points
                val matches = MatOfDMatch()
                matcher.match(
                    mat_from_pts_1c(TRUE_PTS),
                    mat_from_pts_1c(new_pts),
                    matches
                )
                for(match in matches.toArray()) {
                    // Reject outliers
                    if(match.distance > 0.5) continue

                    // Get matched points
                    matched_true_pts.add(TRUE_PTS[match.queryIdx])
                    matched_new_pts.add(new_pts[match.trainIdx])
                }

                println("eaj matched "+matched_new_pts.size)
                if(matched_new_pts.size >= 2) {
                    val move_mat = Calib3d.estimateAffinePartial2D(
                        mat_from_pts(matched_new_pts),
                        mat_from_pts(matched_true_pts),
                    )
                    val new_pos = move_mat * old_pos
                    println("eaj position"+new_pos)
                    old_pos = new_pos
                }
            }


            // Graphic display
            for(pt in new_pts) {
                //Imgproc.drawMarker(warped_img, warp(pt, X, Yf), RED, Imgproc.MARKER_DIAMOND, 18, 1)
                Imgproc.drawMarker(warped_img, map(pt, X, Yf), RED, Imgproc.MARKER_DIAMOND, 18, 1)
            }
            for(pt in matched_new_pts) {
                //Imgproc.drawMarker(warped_img, warp(pt, X, Yf), PINK, Imgproc.MARKER_DIAMOND, 20, 2)
                Imgproc.drawMarker(warped_img, map(pt, X, Yf), PINK, Imgproc.MARKER_DIAMOND, 20, 2)
            }
            for(pt in TRUE_PTS) {
                //Imgproc.circle(warped_img, warp(pt, X, Yf), 6, BLUE, 2)
                Imgproc.circle(warped_img, map(pt, X, Yf), 6, BLUE, 1)
            }
            for(pt in matched_true_pts) {
                //Imgproc.circle(warped_img, warp(pt, X, Yf), 5, CYAN, Imgproc.FILLED)
                Imgproc.circle(warped_img, map(pt, X, Yf), 5, CYAN, Imgproc.FILLED)
            }

            Imgproc.rectangle(warped_img, map(CORNER_POS, X, Yf), map(CORNER_POS * -1.0, X, Yf), BLUE, 1)
            Imgproc.drawMarker(warped_img, map(START_POS, X, Yf), GREEN, Imgproc.MARKER_STAR, 20, 2)
            Imgproc.drawMarker(warped_img, map(old_pos, X, Yf), WHITE, Imgproc.MARKER_CROSS, 20, 2)

            Imgproc.resize(warped_img, warped_img, input.size())

            return warped_img
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