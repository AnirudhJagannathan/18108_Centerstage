package org.firstinspires.ftc.teamcode.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Collections;
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

    private final Scalar WHITE = new Scalar(255, 255, 255);

    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final int WIDTH = 160;
    static final int HEIGHT = 250;
    static final Point r1ATL = new Point(0, 350);
    static final Point r1BTL = new Point(WIDTH, 350 + HEIGHT);
    static final Point r2ATL = new Point(160, 350);
    static final Point r2BTL = new Point(160 + WIDTH, 350 + HEIGHT);
    static final Point r3ATL = new Point(320, 350);
    static final Point r3BTL = new Point(320 + WIDTH, 350 + HEIGHT);
    static final Point r4ATL = new Point(480, 350);
    static final Point r4BTL = new Point(480 + WIDTH, 350 + HEIGHT);
    static final Point r5ATL = new Point(640, 350);
    static final Point r5BTL = new Point(640 + WIDTH, 350 + HEIGHT);
    static final Point r6ATL = new Point(800, 350);
    static final Point r6BTL = new Point(800 + WIDTH, 350 + HEIGHT);
    static final Point r7ATL = new Point(960, 350);
    static final Point r7BTL = new Point(960 + WIDTH, 350 + HEIGHT);
    static final Point r8ATL = new Point(1120, 350);
    static final Point r8BTL = new Point(1120 + WIDTH, 350 + HEIGHT);

    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat Cr = new Mat();
    Mat Y = new Mat();

    Mat region1R;
    Mat region1B;
    int avg1; // The amount of red in the specified rectangle

    // Working variables
    Mat region2R;
    Mat region2B;
    int avg2; // The amount of red in the specified rectangle

    // Working variables
    Mat region3R;
    Mat region3B;
    int avg3; // The amount of red in the specified rectangle

    Mat region4R;
    Mat region4B;
    int avg4; // The amount of red in the specified rectangle

    // Working variables
    Mat region5R;
    Mat region5B;
    int avg5; // The amount of red in the specified rectangle

    // Working variables
    Mat region6R;
    Mat region6B;
    int avg6; // The amount of red in the specified rectangle

    Mat region7R;
    Mat region7B;
    int avg7; // The amount of red in the specified rectangle

    // Working variables
    Mat region8R;
    Mat region8B;
    int avg8; // The amount of red in the specified rectangle

    public volatile Scalar[] vals = new Scalar[8];

    public volatile StackPosition position = StackPosition.POS1;

    // This function takes the RGB frame, converts to YCrCb, and extracts the Cb channel to the 'Cb' variable
    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        region1R = Cb.submat(new Rect(r1ATL, r1BTL));
        region2R = Cb.submat(new Rect(r2ATL, r2BTL));
        region3R = Cb.submat(new Rect(r3ATL, r3BTL));
        region4R = Cb.submat(new Rect(r4ATL, r4BTL));
        region5R = Cb.submat(new Rect(r5ATL, r5BTL));
        region6R = Cb.submat(new Rect(r6ATL, r6BTL));
        region7R = Cb.submat(new Rect(r7ATL, r7BTL));
        region8R = Cb.submat(new Rect(r8ATL, r8BTL));

        inputToCr(firstFrame);
        region1B = Cr.submat(new Rect(r1ATL, r1BTL));
        region2B = Cr.submat(new Rect(r2ATL, r2BTL));
        region3B = Cr.submat(new Rect(r3ATL, r3BTL));
        region4B = Cr.submat(new Rect(r4ATL, r4BTL));
        region5B = Cr.submat(new Rect(r5ATL, r5BTL));
        region6B = Cr.submat(new Rect(r6ATL, r6BTL));
        region7B = Cr.submat(new Rect(r7ATL, r7BTL));
        region8B = Cr.submat(new Rect(r8ATL, r8BTL));
    }

    @Override
    public Mat processFrame(Mat input) {

        Mat areaMat = input.submat(new Rect(r1ATL, r1BTL));
        Scalar sumColors = Core.sumElems(areaMat);
        vals[0] = sumColors;
        areaMat = input.submat(new Rect(r2ATL, r2BTL));
        sumColors = Core.sumElems(areaMat);
        vals[1] = sumColors;
        areaMat = input.submat(new Rect(r3ATL, r3BTL));
        sumColors = Core.sumElems(areaMat);
        vals[2] = sumColors;
        areaMat = input.submat(new Rect(r4ATL, r4BTL));
        sumColors = Core.sumElems(areaMat);
        vals[3] = sumColors;
        areaMat = input.submat(new Rect(r5ATL, r5BTL));
        sumColors = Core.sumElems(areaMat);
        vals[4] = sumColors;
        areaMat = input.submat(new Rect(r6ATL, r6BTL));
        sumColors = Core.sumElems(areaMat);
        vals[5] = sumColors;
        areaMat = input.submat(new Rect(r7ATL, r7BTL));
        sumColors = Core.sumElems(areaMat);
        vals[6] = sumColors;
        areaMat = input.submat(new Rect(r8ATL, r8BTL));
        sumColors = Core.sumElems(areaMat);
        vals[7] = sumColors;

        Scalar max = getMax(vals);

        if (max == vals[0])
            position = StackPosition.POS1;
        if (max == vals[1])
            position = StackPosition.POS2;
        if (max == vals[2])
            position = StackPosition.POS3;
        if (max == vals[3])
            position = StackPosition.POS4;
        if (max == vals[4])
            position = StackPosition.POS5;
        if (max == vals[5])
            position = StackPosition.POS6;
        if (max == vals[6])
            position = StackPosition.POS7;
        if (max == vals[7])
            position = StackPosition.POS8;

        /* inputToCb(input);

        avg1 = (int) Core.mean(region1R).val[0] + (int) Core.mean(region1B).val[0];
        dataFromOpenCV.WHITE1 = avg1;
        vals[0] = avg1;
        avg2 = (int) Core.mean(region2R).val[0] + (int) Core.mean(region2B).val[0];
        dataFromOpenCV.WHITE2 = avg2;
        vals[1] = avg2;
        avg3 = (int) Core.mean(region3R).val[0] + (int) Core.mean(region3B).val[0];
        dataFromOpenCV.WHITE3 = avg3;
        vals[2] = avg3;
        avg4 = (int) Core.mean(region4R).val[0] + (int) Core.mean(region4B).val[0];
        dataFromOpenCV.WHITE4 = avg4;
        vals[3] = avg4;
        avg5 = (int) Core.mean(region5R).val[0] + (int) Core.mean(region5B).val[0];
        dataFromOpenCV.WHITE5 = avg5;
        vals[4] = avg5;
        avg6 = (int) Core.mean(region6R).val[0] + (int) Core.mean(region6B).val[0];
        dataFromOpenCV.WHITE6 = avg6;
        vals[5] = avg6;
        avg7 = (int) Core.mean(region7R).val[0] + (int) Core.mean(region7B).val[0];
        dataFromOpenCV.WHITE7 = avg7;
        vals[6] = avg7;
        avg8 = (int) Core.mean(region8R).val[0] + (int) Core.mean(region8B).val[0];
        dataFromOpenCV.WHITE8 = avg8;
        vals[7] = avg8;
         */

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

        /* int max = Integer.MIN_VALUE;
        int index = 0;

        for (int i = 0; i < 8; i++) {
            if (vals[i] >= max) {
                max = vals[i];
                index = i;
            }
        }

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
         */

        areaMat.release();
        return input;
    }

    private Scalar getMax(Scalar[] scalars) {
        Scalar max = new Scalar(0, 0, 0);
        double avg = 0;
        for (Scalar s : scalars) {
            double currAvg = (s.val[0] + s.val[1] + s.val[2]) / 3;
            if (currAvg > avg) {
                avg = currAvg;
                max = s;
            }
        }
        return max;
    }
}
