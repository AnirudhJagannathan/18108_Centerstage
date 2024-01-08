package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Stack;

public class PixelDetectionPipeline extends OpenCvPipeline {

    public enum StackPosition {
        POS1,
        POS2,
        POS3,
        POS4,
        POS5,
        POS6,
        POS7,
        POS8;
    }

    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final int WIDTH = 135;
    static final int HEIGHT = 500;
    static final Point r1ATL = new Point(0, 100);
    static final Point r1BTL = new Point(WIDTH, 100 + HEIGHT);
    static final Point r2ATL = new Point(135, 100);
    static final Point r2BTL = new Point(135 + WIDTH, 100 + HEIGHT);
    static final Point r3ATL = new Point(270, 100);
    static final Point r3BTL = new Point(270 + WIDTH, 100 + HEIGHT);
    static final Point r4ATL = new Point(405, 100);
    static final Point r4BTL = new Point(405 + WIDTH, 100 + HEIGHT);
    static final Point r5ATL = new Point(540, 100);
    static final Point r5BTL = new Point(540 + WIDTH, 100 + HEIGHT);
    static final Point r6ATL = new Point(675, 100);
    static final Point r6BTL = new Point(675 + WIDTH, 100 + HEIGHT);
    static final Point r7ATL = new Point(810, 100);
    static final Point r7BTL = new Point(810 + WIDTH, 100 + HEIGHT);
    static final Point r8ATL = new Point(945, 100);
    static final Point r8BTL = new Point(945 + WIDTH, 100 + HEIGHT);

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat Cr = new Mat();

    Mat region1;
    int avg1; // The amount of red in the specified rectangle

    // Working variables
    Mat region2;
    int avg2; // The amount of red in the specified rectangle

    // Working variables
    Mat region3;
    int avg3; // The amount of red in the specified rectangle

    Mat region4;
    int avg4; // The amount of red in the specified rectangle

    // Working variables
    Mat region5;
    int avg5; // The amount of red in the specified rectangle

    // Working variables
    Mat region6;
    int avg6; // The amount of red in the specified rectangle

    Mat region7;
    int avg7; // The amount of red in the specified rectangle

    // Working variables
    Mat region8;
    int avg8; // The amount of red in the specified rectangle

    int[] vals = new int[8];

    public volatile StackPosition position = StackPosition.POS1;

    // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1 = Cb.submat(new Rect(r1ATL, r1BTL));
        region2 = Cb.submat(new Rect(r2ATL, r2BTL));
        region3 = Cb.submat(new Rect(r3ATL, r3BTL));
        region4 = Cb.submat(new Rect(r4ATL, r4BTL));
        region5 = Cb.submat(new Rect(r5ATL, r5BTL));
        region6 = Cb.submat(new Rect(r6ATL, r6BTL));
        region7 = Cb.submat(new Rect(r7ATL, r7BTL));
        region8 = Cb.submat(new Rect(r8ATL, r8BTL));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1).val[0];
        dataFromOpenCV.WHITE1 = avg1;
        vals[0] = avg1;
        avg2 = (int) Core.mean(region2).val[0];
        dataFromOpenCV.WHITE2 = avg2;
        vals[1] = avg2;
        avg3 = (int) Core.mean(region3).val[0];
        dataFromOpenCV.WHITE3 = avg3;
        vals[2] = avg3;
        avg4 = (int) Core.mean(region4).val[0];
        dataFromOpenCV.WHITE4 = avg4;
        vals[3] = avg4;
        avg5 = (int) Core.mean(region5).val[0];
        dataFromOpenCV.WHITE5 = avg5;
        vals[4] = avg5;
        avg6 = (int) Core.mean(region6).val[0];
        dataFromOpenCV.WHITE6 = avg6;
        vals[5] = avg6;
        avg7 = (int) Core.mean(region7).val[0];
        dataFromOpenCV.WHITE7 = avg7;
        vals[6] = avg7;
        avg8 = (int) Core.mean(region8).val[0];
        dataFromOpenCV.WHITE8 = avg8;
        vals[7] = avg8;

        Imgproc.rectangle(
                input, // Buffer to draw on
                r1ATL, // First point which defines the rectangle
                r1BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r2ATL, // First point which defines the rectangle
                r2BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r3ATL, // First point which defines the rectangle
                r3BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r4ATL, // First point which defines the rectangle
                r4BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r5ATL, // First point which defines the rectangle
                r5BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r6ATL, // First point which defines the rectangle
                r6BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r7ATL, // First point which defines the rectangle
                r7BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        Imgproc.rectangle(
                input, // Buffer to draw on
                r8ATL, // First point which defines the rectangle
                r8BTL, // Second point which defines the rectangle
                GREEN, // The color the rectangle is drawn in
                3); // Thickness of the rectangle lines

        int max = Integer.MIN_VALUE;

        for (int i = 0; i < 8; i++) {
            if (vals[i] > max)
                max = vals[i];
        }

        int index = Arrays.asList(vals).indexOf(max);

        if (index == 0)
            position = StackPosition.POS1;
        if (index == 1)
            position = StackPosition.POS2;
        if (index == 2)
            position = StackPosition.POS3;
        if (index == 3)
            position = StackPosition.POS4;
        if (index == 4)
            position = StackPosition.POS5;
        if (index == 5)
            position = StackPosition.POS6;
        if (index == 6)
            position = StackPosition.POS7;
        if (index == 7)
            position = StackPosition.POS8;

        return input;
    }
}
