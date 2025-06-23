package org.firstinspires.ftc.teamcode.vision;

import android.annotation.SuppressLint;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class SampleAngleProcessor implements VisionProcessor {

    private Mat cameraMatrix, distCoeffs;
    private double blockAngle = 0;
    private boolean foundBlock = false;

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // === Undistort ===
        Mat undistorted = new Mat();
        Calib3d.undistort(input, undistorted, cameraMatrix, distCoeffs);

        // === Convert to HSV ===
        Mat hsv = new Mat();
        Imgproc.cvtColor(undistorted, hsv, Imgproc.COLOR_RGB2HSV);

        // === Color thresholding for red (adjust as needed) ===
        Mat mask1 = new Mat();
        Mat mask2 = new Mat();

        Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), mask1);      // red lower range
        Core.inRange(hsv, new Scalar(160, 100, 100), new Scalar(179, 255, 255), mask2);   // red upper range

        Mat mask = new Mat();
        Core.bitwise_or(mask1, mask2, mask);

        // === Morphological cleanup ===
        Imgproc.erode(mask, mask, new Mat(), new org.opencv.core.Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new org.opencv.core.Point(-1, -1), 2);

        // === Find contours ===
        java.util.List<MatOfPoint> contours = new java.util.ArrayList<>();
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        foundBlock = false;

        if (!contours.isEmpty()) {
            // Find largest contour
            MatOfPoint largest = contours.get(0);
            double maxArea = Imgproc.contourArea(largest);

            for (MatOfPoint c : contours) {
                double area = Imgproc.contourArea(c);
                if (area > maxArea) {
                    maxArea = area;
                    largest = c;
                }
            }

            if (maxArea > 500) {  // Only consider reasonably large objects
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largest.toArray()));

                blockAngle = rect.angle;

                // Normalize based on size to get consistent 0-90 behavior
                if (rect.size.width < rect.size.height) {
                    blockAngle = rect.angle + 90;  // Vertical block = 90°
                } else {
                    blockAngle = rect.angle;       // Flat block = 0°
                }

                // Ensure positive angles only
                if (blockAngle < 0) {
                    blockAngle += 180;
                }
                foundBlock = true;

                // Draw box on input
                Point[] box = new Point[4];
                rect.points(box);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(undistorted, box[i], box[(i + 1) % 4], new Scalar(0, 255, 0), 2);
                }

                // Draw angle text
                Imgproc.putText(undistorted,
                        String.format("Angle: %.1f deg", blockAngle),
                        rect.center,
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        new Scalar(255, 255, 0),
                        2);
            }
        }

        undistorted.copyTo(input);
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
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(0xFF00FF00);
        paint.setTextSize(50);

        if (foundBlock) {
            canvas.drawText(String.format("Block Angle: %.1f°", blockAngle), 40, 60, paint);
        } else {
            canvas.drawText("No block detected", 40, 60, paint);
        }
    }

    // === Optional Getter ===
    public double getBlockAngle() {
        return blockAngle;
    }

    public boolean isBlockFound() {
        return foundBlock;
    }
}
