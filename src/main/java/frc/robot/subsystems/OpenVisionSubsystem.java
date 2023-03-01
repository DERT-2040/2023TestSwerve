package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

public class OpenVisionSubsystem extends SubsystemBase{
    public OpenVisionSubsystem() {
        //Vision Device Setup
        cap = new VideoCapture(0);
        //Importent Outputs
        alignmentOutput = 0;
        roti_pass = false;
        //Program Settings
        erodecycles = 1;
        blurradius = 5;
        vertical_threshold = 230;
        alignment_error_thershold = 30;
        //Setup Camera Device
        cap.set(Videoio.CAP_PROP_EXPOSURE, -11);
        cap.set(Videoio.CAP_PROP_CONTRAST, 13);
    }
    static VideoCapture cap;
    static int alignmentOutput;
    static boolean roti_pass;
    static int erodecycles;
    static int blurradius;
    static int vertical_threshold;
    static int alignment_error_thershold;
    static Mat eroded_image;
    static Mat eroded_image_roti;
    static Mat eroded_image_alin;
    static Mat eroded_image_multi;
    static int white_stop;
    public static void ProcessVision() {
        //
        // Image Processing
        //
        Mat frame = new Mat();
        cap.read(frame);
        //Apply Box Blur
        Mat blurred_image = new Mat();
        Imgproc.blur(frame, blurred_image, new Size(blurradius, blurradius));
        //Convert to HSV
        Mat hsv_img = new Mat();
        Imgproc.cvtColor(blurred_image, hsv_img, Imgproc.COLOR_BGR2HSV);
        //Define HSV Filter Settings
        Scalar lower_threshold1 = new Scalar(16, 140, 87);
        Scalar upper_threshold1 = new Scalar(47, 215, 225);
        Scalar lower_threshold2 = new Scalar(0, 0, 255);
        Scalar upper_threshold2 = new Scalar(0, 0, 255);
        //Apply HSV Filters
        Mat threshold1_mask = new Mat();
        Core.inRange(hsv_img, lower_threshold1, upper_threshold1, threshold1_mask);
        Mat threshold2_mask = new Mat();
        Core.inRange(hsv_img, lower_threshold2, upper_threshold2, threshold2_mask);
        //Merge Masks
        Mat combinedMask = new Mat();
        Core.bitwise_or(threshold1_mask, threshold2_mask, combinedMask);
        //Define Erosion Structure
        Mat kernel_Mat = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        //Preform Erosion
        Imgproc.erode(combinedMask, eroded_image, kernel_Mat, new Point(-1, -1), erodecycles);
        //Clone Images (Multi-Channel Image is not needed)
        eroded_image_multi = eroded_image.clone();
        Imgproc.cvtColor(eroded_image_multi, eroded_image_multi, Imgproc.COLOR_GRAY2BGR);
        eroded_image_alin = eroded_image.clone();
        eroded_image_roti = eroded_image.clone();
        //Stops if no white pixels
        white_stop = Core.countNonZero(eroded_image);
    }
    public static boolean CheckTurntable() {
        int roti_white_stop = Core.countNonZero(eroded_image_roti);
        if ((white_stop != 0) && (roti_white_stop != 0)) {
            //
            // Cone Turntable Rotation
            //
            //Sets the upper image section to black
            Point rotiStart = new Point(0, 0);
            Point rotiEnd = new Point(eroded_image_roti.cols(), (eroded_image_roti.rows()-vertical_threshold));
            Imgproc.rectangle(eroded_image_roti, rotiStart, rotiEnd, new Scalar(0, 0, 0), -1);
            //List of contours
            List<MatOfPoint> turntableContours = new ArrayList<>();
            //Find and sort contours
            Mat hierarchey = new Mat();
            Imgproc.findContours(eroded_image_roti, turntableContours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double roti_area = 0;
            int roti_idx = 0;
            for (int i = 0; i < turntableContours.size(); i++) {
                double tempContourArea = Imgproc.contourArea(turntableContours.get(i));
                if (tempContourArea > roti_area) {
                    roti_area = tempContourArea;
                    roti_idx = i;
                }
            }
            //Calculate the bounding rect of the contour "idx"
            Rect largestRect = Imgproc.boundingRect(turntableContours.get(roti_idx));
            //Find the corners (Not needed)
            org.opencv.core.Point pt1 = new org.opencv.core.Point(largestRect.x, largestRect.y);
            org.opencv.core.Point pt2 = new org.opencv.core.Point(largestRect.x + largestRect.width, largestRect.y + largestRect.height);
            //Draw the Rectangle (Not Neeeded)
            Imgproc.rectangle(eroded_image_multi, pt1, pt2, new Scalar(255, 0, 0));
            //Pass or Fail math
            double boxRatio = ((double)largestRect.width / largestRect.height);
            int box_pixel_size = largestRect.height * largestRect.width;
            double ratio = (Imgproc.contourArea(turntableContours.get(roti_idx))) / box_pixel_size;
            boolean pass_bw = false;
            boolean pass_box_size = false;
            if (ratio >= 0.48) {
                pass_bw = true;
            } else {
                pass_bw = false;
            }
            if (boxRatio <= 1.1 && largestRect.width >= 125) {
                pass_box_size = true;
            } else {
                pass_box_size = false;
            }
            if (pass_bw && pass_box_size) {
                roti_pass = true;
            } else {
                roti_pass = false;
            }
            //Put the status on screen (not needed)
            double textScale = 1;
            if (roti_pass) {
                Imgproc.putText(eroded_image_multi, "Pass", new Point(25, 30), 2, textScale, new Scalar(0, 230, 60), 2);
            } else {
                Imgproc.putText(eroded_image_multi, "Fail", new Point(25, 30), 2, textScale, new Scalar(200, 10, 0), 2);
            }
            if (pass_bw) {
                Imgproc.putText(eroded_image_multi, "Pass B&W", new Point(25, 70), 2, textScale, new Scalar(60, 130, 0), 2);
            } else {
                Imgproc.putText(eroded_image_multi, "Fail B&W", new Point(25, 70), 2, textScale, new Scalar(200, 175, 0), 2);
            }
            if (pass_box_size) {
                Imgproc.putText(eroded_image_multi, "Pass Box Size", new Point(25, 110), 2, textScale, new Scalar(60, 130, 0), 2);
            } else {
                Imgproc.putText(eroded_image_multi, "Fail Box Size", new Point(25, 110), 2, textScale, new Scalar(200, 175, 0), 2);
            }
        }
        return roti_pass;
    }
    public static int CheckRobotAlignment() {
        int alin_white_stop = Core.countNonZero(eroded_image_alin);
        if ((white_stop != 0) && (alin_white_stop != 0)) {
            //
            // Robot Alignment
            //
            //Sets the bottom section to black
            Point alinStart = new Point(0, (eroded_image_alin.rows() - vertical_threshold));
            Point alinEnd = new Point(eroded_image_alin.cols(), eroded_image_alin.rows());
            Imgproc.rectangle(eroded_image_alin, alinStart, alinEnd, new Scalar(0, 0, 0), -1);
            //List of contours
            List<MatOfPoint> alignmentContours = new ArrayList<>();
            //Find and sort contours
            Mat hierarchey = new Mat();
            Imgproc.findContours(eroded_image_alin, alignmentContours, hierarchey, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double alin_area = 0;
            int alin_idx = 0;
            for (int i = 0; i < alignmentContours.size(); i++) {
                double tempContourArea = Imgproc.contourArea(alignmentContours.get(i));
                if (tempContourArea > alin_area) {
                    alin_area = tempContourArea;
                    alin_idx = i;
                }
            }
            if (alignmentContours.size() != 0) {
                //Find Size of the enclosing circle
                MatOfPoint2f alignmentContours_2f = new MatOfPoint2f(alignmentContours.get(alin_idx).toArray());
                float[] circle_radius = {0};
                Point firstCords = new Point();
                Imgproc.minEnclosingCircle(alignmentContours_2f, firstCords, circle_radius);
                //Calculate Output
                if (firstCords.x >= (eroded_image_alin.cols() / 2 + alignment_error_thershold)) {
                    alignmentOutput = 3;
                } else if (firstCords.x <= (eroded_image_alin.cols() / 2 - alignment_error_thershold)) {
                    alignmentOutput = 1;
                } else if (firstCords.x < (eroded_image_roti.cols() / 2 + alignment_error_thershold) || firstCords.x >(eroded_image_alin.cols() / 2 - alignment_error_thershold)) {
                    alignmentOutput = 2;
                }
                //Print the output on the screen (Not Needed)
                double textScale = 1;
                if (alignmentContours.size() != 0) {
                    Imgproc.circle(eroded_image_multi, firstCords, (int) circle_radius[0], new Scalar(55, 40, 240), 2);
                }
                if (alignmentOutput == 3) {
                    Imgproc.putText(eroded_image_multi, "Turn -->", new Point(425, 30), 2, textScale, new Scalar(240, 170, 10), 2);
                }
                if (alignmentOutput == 1) {
                    Imgproc.putText(eroded_image_multi, "Turn <--", new Point(425, 30), 2, textScale, new Scalar(240, 170, 10), 2);
                }
                if (alignmentOutput == 2) {
                    Imgproc.putText(eroded_image_multi, "Correct", new Point(425, 30), 2, textScale, new Scalar(20, 190, 20), 2);
                }
            }
        }
        return alignmentOutput;
    }
    public static Mat getErodedImageMulti() {
        return eroded_image_multi;
    }
}
