package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.PIDConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class PixelTriangulationPipeline extends OpenCvPipeline
{
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;


    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private BNO055IMU imu;
    static double cX = 0;
    static double cY = 0;
    static double width = 0;
    static double cX2 = 0;
    static double cY2 = 0;
    static double width2 = 0;

    Mat yellowMask;
    List<MatOfPoint> contours;
    Mat hierarchy = null;
    MatOfPoint largestContour = null;
    MatOfPoint nextLargestContour = null;
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    public static final int CAMERA_WIDTH = 1920; // width  of wanted camera resolution
    public static final int CAMERA_HEIGHT = 1080; // height of wanted camera resolution
    private static final double FOV = 46.4;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 1425;  // Replace with the focal length of the camera in pixels

    Scalar GREEN = new Scalar(0, 0, 255);

    Mat region1;
    Mat region2;

    Point point1A = new Point(225, 400);
    Point point1B = new Point(350, 500);

    Point point2A = new Point(550, 400);
    Point point2B = new Point(675, 500);

    Mat hsv = new Mat();

    double avg1, avg2;

    @Override
    public void init(Mat firstFrame) {
        Imgproc.cvtColor(firstFrame, hsv, Imgproc.COLOR_RGB2HSV);

        region1 = hsv.submat(new Rect(point1A, point1B));
        region2 = hsv.submat(new Rect(point2A, point2B));
    }

    @Override
    public Mat processFrame(Mat input) {
        // Preprocess the frame to detect white regions

        // yellowMask = preprocessFrame(input);

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2BGR);

        avg1 = Math.sqrt(Math.pow(Core.mean(region1).val[0], 2) + Math.pow(Core.mean(region1).val[1], 2) + Math.pow(Core.mean(region1).val[2], 2));
        avg2 = Math.sqrt(Math.pow(Core.mean(region2).val[0], 2) + Math.pow(Core.mean(region2).val[1], 2) + Math.pow(Core.mean(region2).val[2], 2));

        dataFromOpenCV.AVG1W = avg1;
        dataFromOpenCV.AVG2W = avg2;

        Imgproc.rectangle(
                input, // Buffer to draw on
                point1A, // First point which defines the rectangle
                point1B, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                point2A, // First point which defines the rectangle
                point2B, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines


        // Find contours of the detected yellow regions

        /*
        contours = new ArrayList<>();
        hierarchy = new Mat();
        Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest yellow contour (blob)
        largestContour = findLargestContour(contours);
        nextLargestContour = findNextLargestContour(contours);

        if (largestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box

            width = 1.2533*calculateWidth(largestContour);

            // Display the width next to the label
            String widthLabel = "Width1: " + (int) width + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(largestContour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX + ", " + (int) cY + ")";
            Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
        }

        if (nextLargestContour != null) {
            // Draw a red outline around the largest detected object
            Imgproc.drawContours(input, contours, contours.indexOf(nextLargestContour), new Scalar(255, 0, 0), 2);
            // Calculate the width of the bounding box

            width2 = 1.2533*calculateWidth(nextLargestContour);

            // Display the width next to the label
            String widthLabel = "Width2: " + (int) width2 + " pixels";
            Imgproc.putText(input, widthLabel, new Point(cX2 + 10, cY2 + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            //Display the Distance
            String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width2)) + " inches";
            Imgproc.putText(input, distanceLabel, new Point(cX2 + 10, cY2 + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            // Calculate the centroid of the largest contour
            Moments moments = Imgproc.moments(nextLargestContour);
            cX2 = moments.get_m10() / moments.get_m00();
            cY2 = moments.get_m01() / moments.get_m00();

            // Draw a dot at the centroid
            String label = "(" + (int) cX2 + ", " + (int) cY2 + ")";
            Imgproc.putText(input, label, new Point(cX2 + 10, cY2), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            Imgproc.circle(input, new Point(cX2, cY2), 5, new Scalar(0, 255, 0), -1);
        }
         */

        return input;
    }

    private Mat preprocessFrame(Mat frame) {
        Mat hsvFrame = new Mat();
        Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            /* Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255);
             */

        Scalar lowerYellow = new Scalar(0, 0, 240);
        Scalar upperYellow = new Scalar(255, 15, 255);


        Mat yellowMask = new Mat();
        Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

        return yellowMask;
    }

    public static MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }

        return largestContour;
    }

    public static MatOfPoint findNextLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;
        MatOfPoint nextLargestContour = null;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                nextLargestContour = largestContour;
                largestContour = contour;
            }
        }

        return nextLargestContour;
    }

    public static double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    private static double getAngleTarget(double objMidpoint){
        double midpoint = -((objMidpoint - (CAMERA_WIDTH/2))*FOV)/CAMERA_WIDTH;
        return midpoint;
    }
    public static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
    public double PIDControl(double refrence, double state) {
        double error = angleWrap(refrence - state);
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

    public static double getWidth() {
        return width;
    }

}
