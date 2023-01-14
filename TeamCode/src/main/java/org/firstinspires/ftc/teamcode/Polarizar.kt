package org.firstinspires.ftc.teamcode

import org.opencv.core.*
import org.opencv.features2d.BFMatcher
import org.opencv.imgproc.Imgproc
import org.opencv.calib3d.Calib3d
import org.openftc.easyopencv.OpenCvPipeline

class PolarizarPipeline : OpenCvPipeline() {
    val START_POS = Point(1.65, 1.82)
    val CORNER_POS = Point(3.0, 3.0)
    val GRID_SIZE = 185
    val TRUE_PTS = arrayOf(
        Point(-1.0, -2.0), Point(+1.0, -2.0),
        Point(-2.0, -1.0), Point(-1.0, -1.0), Point(0.0, -1.0), Point(+1.0, -1.0), Point(+2.0, -1.0),
        Point(-1.0, 0.0), Point(+1.0, 0.0),
        Point(-2.0, +1.0), Point(-1.0, +1.0), Point(0.0, +1.0), Point(+1.0, +1.0), Point(+2.0, +1.0),
        Point(-1.0, +2.0), Point(+1.0, +2.0)

    )
    val MAP_SIZE = 300

    val matcher = BFMatcher(Core.NORM_L2, true)

    var old_pos = START_POS

    infix fun Point.plus(a: Point) = Point(this.x + a.x, this.y + a.y)
    infix fun Point.minus(a: Point) = Point(this.x - a.x, this.y - a.y)
    infix fun Point.dot(a: Point) = this.x * a.x + this.y * a.y
    infix fun Point.times(a: Double) = Point(this.x * a, this.y * a)

    infix fun Mat.times(a: Point): Point {
        return Point(
            this.get(0,0)[0] * a.x + this.get(0, 1)[0] * a.y + this.get(0,2)[0],
            this.get(1,0)[0] * a.x + this.get(1, 1)[0] * a.y + this.get(1,2)[0]
        )
    }

    private fun mat_from_pts(pts: List<Point>): Mat {
        val m = Mat(pts.size, 1, CvType.CV_32FC2)
        for(i in pts.indices) {
            m.put(2*i, 0, pts[i].x, pts[i].y)
        }
        return m
    }
    private fun mat_from_pts(vararg pts: Point): Mat {
        return mat_from_pts(pts.asList())
    }
    private fun pts_from_mat(m: Mat): Array<Point> {
        val pts = Array(m.rows()/2) {
            Point(m.get(it, 0)[0], m.get(it+1, 0)[0])
        }
        return pts
    }
    private fun seg_intersect(a1: Point, a2: Point, b1: Point, b2: Point): Point {
        val da = a2 minus a1
        val db = b2 minus b1
        val dp = a1 minus b1
        val dap = Point(-da.y, da.x)
        val denom = dap dot db
        val num = dap dot dp
        return (db times (num / denom)) plus b1
    }

    override fun processFrame(input: Mat): Mat {
        try {
            println("-----PROCESS FRAME " + System.currentTimeMillis())

            //        Core.rotate(input, img, Core.ROTATE_90_CLOCKWISE)

            val hsv = input.clone()
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV)
            val lower = S(10, 120, 50)
            val upper = S(25, 255, 255)
            val mask = Mat()
            Core.inRange(hsv, lower, upper, mask)

            val Y = input.rows().toDouble()
            val X = input.cols().toDouble()
            val w = 1500.0
            val Yf = Y * 1.5
            val src_plane = mat_from_pts(
                Point(0.0, 0.0),
                Point(X, 0.0),
                Point(X + w, Y),
                Point(- w, Y),
            )
            val project_plane = mat_from_pts(
                Point(0.0, 0.0),
                Point(X, 0.0),
                Point(X, Y),
                Point(0.0, Y)
            )

            val project_mat = Imgproc.getPerspectiveTransform(src_plane, project_plane)
            val warped_img = Mat()
            Imgproc.warpPerspective(input, warped_img, project_mat, input.size())

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
                if (area < 100) continue

                println("eaj a valid contour ")

                val contour_pts = contour.toArray()

                // Find convex hull of contour (fills noise within pole)
                val hull = MatOfInt()
                Imgproc.convexHull(contour, hull)
                val hull_pts = Array<Point>(hull.rows()) {
                    contour_pts[it]
                }

                // Find bounding rotated rectangle of contour (useful for extent of pole)
                val box = Imgproc.minAreaRect(MatOfPoint2f(*hull_pts))
                val box_pts = Array<Point?>(4) { null }
                box.points(box_pts)
                box_pts.sortByDescending { it?.y }

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
                val line_pt = Point(line.get(2, 0)[0], line.get(3, 0)[0])

                // Calculate base of hull as intersection between the line of best fit passing through
                // the center of mass and the bottom edge of the bounding box
                val left = box_pts[0]!!
                val right = box_pts[1]!!
                val base = seg_intersect(center, center plus slope, left, right)

                println("eaj base" + base)
                // Store attributes of pole
                val pole = mat_from_pts(base, left, right, center)
                println("eaj pole " + pole.dump().replace("\n",";"))
                println("eaj proj  " + project_mat.dump().replace("\n",";"))

                // Perspective warp pole features
                val warped_pole = Mat(4, 1, CvType.CV_32FC2)
                Core.perspectiveTransform(pole, warped_pole, project_mat)
                println("eaj warped pole"+warped_pole.dump().replace("\n",";"))
                val (warped_base, warped_left, warped_right, warped_center) = pts_from_mat(warped_pole)
                println("eaj deconstructed warped pole" + warped_base)

                // Camera is at center bottom of input view
                val rel_pos = Point(X / 2, Yf) minus warped_base
                val new_pt = old_pos minus rel_pos

                new_pts.add(new_pt)

                Imgproc.line(input, base, center, BLUE, 2)
                Imgproc.line(warped_img, warped_base, warped_center, BLUE, 2)

                Imgproc.circle(input, base, 10, GREEN, 2)
                Imgproc.circle(warped_img, warped_base, 10, GREEN, 2)

                Imgproc.line(input, left, right, BLUE, 2)
                Imgproc.line(warped_img, warped_left, warped_right, BLUE, 2)

                println("eaj a pole" + warped_base)
            }

            if (new_pts.size >= 2) {
                // Match up true and observed points
                val matches = MatOfDMatch()
                matcher.match(
                    mat_from_pts(*TRUE_PTS),
                    mat_from_pts(*new_pts.toTypedArray()),
                    matches
                )
                val matched_true_pts = arrayListOf<Point>()
                val matched_new_pts = arrayListOf<Point>()
                for(match in matches.toArray()) {
                    // Reject outliers
                    if(match.distance > 0.5) continue

                    // Get matched points
                    matched_true_pts.add(TRUE_PTS[match.queryIdx])
                    matched_new_pts.add(new_pts[match.trainIdx])
                }

                if(matched_new_pts.size >= 2) {
                    val move_mat = Calib3d.estimateAffinePartial2D(
                        mat_from_pts(matched_new_pts),
                        mat_from_pts(matched_true_pts),
                    )
                    val new_pos = move_mat times old_pos
                    print(new_pos)
                }
            }


            return input
        } catch (e: Exception) {
            println("eaj exception " + e.toString().replace("\n", " \\n "))
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