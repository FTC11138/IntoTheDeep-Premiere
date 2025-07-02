package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.NewIntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleAngleProcessor implements VisionProcessor {

    private Mat cameraMatrix, distCoeffs;
    private double blockAngle = 0;
    private boolean foundBlock = false;
    private double dx = 0;
    private double dy = 0;

    private int imageWidth = 0;
    private int imageHeight = 0;

    public final Scalar redLow1 = new Scalar(0,100, 100);
    public final Scalar redHigh1 = new Scalar(10, 255, 255);
    public final Scalar redLow2 = new Scalar(160,100, 100);
    public final Scalar redHigh2 = new Scalar(179, 255, 255);

    public final Scalar yellowLow = new Scalar(20, 100, 100);
    public final Scalar yellowHigh = new Scalar(35, 255, 255);


    public final Scalar blueLow = new Scalar(100, 150, 50);
    public final Scalar blueHigh = new Scalar(130, 255, 255);

    private static Mat hsv;
    private static Mat mask;
    private static Mat redMask1, redMask2;
    private static Mat yellowMask;
    private static Mat blueMask;


    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        mask = new Mat();

        // Red Mask
        redMask1 = new Mat();
        redMask2 = new Mat();
        Core.inRange(hsv, redLow1, redHigh1, redMask1);
        Core.inRange(hsv, redLow2, redHigh2, redMask2);
        Core.bitwise_or(redMask1, redMask2, mask);

        // Yellow Mask
        yellowMask = new Mat();
        Core.inRange(hsv, yellowLow, yellowHigh, yellowMask);
        Core.bitwise_or(mask, yellowMask, mask);

        // Blue Mask
        blueMask = new Mat();
        Core.inRange(hsv, blueLow, blueHigh, blueMask);
        Core.bitwise_or(mask, blueMask, mask);

        redMask1.release();
        redMask2.release();
        yellowMask.release();
        blueMask.release();

        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        foundBlock = false;
        double minDistance = Double.MAX_VALUE;
        RotatedRect closestRect = null;

        double centerX = imageWidth / 2.0;
        double centerY = imageHeight / 2.0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 500) {
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                double rectCenterX = rect.center.x;
                double rectCenterY = rect.center.y;
                double distance = Math.hypot(rectCenterX - centerX, rectCenterY - centerY);

                if (distance < minDistance) {
                    minDistance = distance;
                    closestRect = rect;
                }
            }
        }

        if (closestRect != null) {
            double width = closestRect.size.width;
            double height = closestRect.size.height;
            blockAngle = closestRect.angle;

            if (width < height) {
                blockAngle = blockAngle + 90;
            }
            if (blockAngle < 0) {
                blockAngle += 180;
            }

            dx = closestRect.center.x - centerX;
            dy = closestRect.center.y - centerY;
            foundBlock = true;

            Point[] box = new Point[4];
            closestRect.points(box);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, box[i], box[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            Imgproc.putText(input,
                    String.format("Angle: %.1f deg dx: %.1f dy: %.1f", blockAngle, dx, dy),
                    closestRect.center,
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    new Scalar(255, 255, 0),
                    2);
        }

        input.copyTo(input);
        return null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        distCoeffs = new Mat(1, 5, CvType.CV_64F);

        cameraMatrix.put(0, 0,
                Constants.fX, 0, Constants.cX,
                0, Constants.fY, Constants.cY,
                0, 0, 1
        );
        distCoeffs.put(0, 0,
                Constants.k1, Constants.k2, Constants.p1, Constants.p2, Constants.k3);

        imageWidth = width;
        imageHeight = height;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(0xFF00FF00);
        paint.setTextSize(50);

        if (foundBlock) {
            canvas.drawText(String.format("Angle: %.1f° dx: %.1f dy: %.1f", blockAngle, dx, dy), 40, 60, paint);
        } else {
            canvas.drawText("No block detected", 40, 60, paint);
        }
    }

    // === Getters ===
    public double getBlockAngle() {
        return blockAngle;
    }

    public boolean isBlockFound() {
        return foundBlock;
    }

    public double getDx() {
        return dx;
    }

    public double getDy() {

        double newDy = dy;
        if (Robot.getInstance().newIntakeSubsystem.wristState == NewIntakeSubsystem.WristState.PREGRABBACK) {
            newDy -= Constants.extGrabBackOffset;
        } else if (Robot.getInstance().newIntakeSubsystem.wristState == NewIntakeSubsystem.WristState.PREGRABFORWARD) {
            newDy += Constants.extGrabFowardOffset;
        }
        newDy *= Constants.sampleDyCorrectionMultiplier;

        return newDy;
    }
}
