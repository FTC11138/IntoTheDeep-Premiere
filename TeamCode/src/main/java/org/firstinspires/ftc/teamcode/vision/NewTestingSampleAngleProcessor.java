package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

import com.acmerobotics.dashboard.config.Config;

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
import org.opencv.imgproc.*;

import java.util.ArrayList;
import java.util.List;

@Config
public class NewTestingSampleAngleProcessor implements VisionProcessor {

    // ===== Bird's-eye / undistort =====
    public static boolean USE_UNDISTORT = true;
    public static boolean SHOW_OVERLAY = true;

    // Source corners in RAW camera image (order: TL, TR, BR, BL)
    public static double SRC_TL_X = 200, SRC_TL_Y = 120;
    public static double SRC_TR_X = 1180, SRC_TR_Y = 110;
    public static double SRC_BR_X = 1230, SRC_BR_Y = 680;
    public static double SRC_BL_X = 170,  SRC_BL_Y = 700;

    // Output flat-view size
    public static int WARP_WIDTH = 900;
    public static int WARP_HEIGHT = 600;

    // Search Region (ROI) in the FLAT view
    public static int ROI_X = 0;
    public static int ROI_Y = 0;
    public static int ROI_WIDTH = 900;
    public static int ROI_HEIGHT = 600;

    // ===== Color thresholds =====
    public static Scalar redLow1 = new Scalar(0, 70, 50);
    public static Scalar redHigh1 = new Scalar(15, 255, 255);
    public static Scalar redLow2 = new Scalar(150, 70, 50);
    public static Scalar redHigh2 = new Scalar(179, 255, 255);

    public static Scalar yellowLow = new Scalar(20, 0, 100);
    public static Scalar yellowHigh = new Scalar(60, 255, 255);

    public static Scalar blueLow = new Scalar(100, 150, 100);
    public static Scalar blueHigh = new Scalar(130, 255, 255);

    // ===== Internals =====
    private Mat cameraMatrix, distCoeffs;
    private Mat homography; // 3x3
    private boolean homographyDirty = true;

    private static Mat hsv;
    private static Mat mask;
    private static Mat redMask1, redMask2;
    private static Mat yellowMask;
    private static Mat blueMask;

    private double blockAngle = 0;
    private boolean foundBlock = false;
    private Globals.COLORS blockColor = Globals.COLORS.NONE;
    private double dx = 0;
    private double dy = 0;

    private int imageWidth = 0;
    private int imageHeight = 0;

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        // 1) Optional undistort (no rotate/flip; camera is fixed)
        Mat undistorted = new Mat();
        if (USE_UNDISTORT && cameraMatrix != null && !cameraMatrix.empty()) {
            Calib3d.undistort(input, undistorted, cameraMatrix, distCoeffs);
        } else {
            undistorted = input;
        }

        // 2) Build/update homography
        if (homography == null || homographyDirty) {
            MatOfPoint2f src = new MatOfPoint2f(
                    new Point(SRC_TL_X, SRC_TL_Y),
                    new Point(SRC_TR_X, SRC_TR_Y),
                    new Point(SRC_BR_X, SRC_BR_Y),
                    new Point(SRC_BL_X, SRC_BL_Y)
            );
            MatOfPoint2f dst = new MatOfPoint2f(
                    new Point(0, 0),
                    new Point(WARP_WIDTH - 1, 0),
                    new Point(WARP_WIDTH - 1, WARP_HEIGHT - 1),
                    new Point(0, WARP_HEIGHT - 1)
            );
            homography = Imgproc.getPerspectiveTransform(src, dst);
            homographyDirty = false;

            imageWidth = WARP_WIDTH;
            imageHeight = WARP_HEIGHT;

            // default ROI to full frame if out of bounds
            ROI_X = clamp(ROI_X, 0, imageWidth);
            ROI_Y = clamp(ROI_Y, 0, imageHeight);
            ROI_WIDTH = clamp(ROI_WIDTH, 1, imageWidth - ROI_X);
            ROI_HEIGHT = clamp(ROI_HEIGHT, 1, imageHeight - ROI_Y);
        }

        // 3) Warp to flat view
        Mat topDown = new Mat();
        Imgproc.warpPerspective(
                undistorted,
                topDown,
                homography,
                new org.opencv.core.Size(WARP_WIDTH, WARP_HEIGHT),
                Imgproc.INTER_LINEAR,
                Core.BORDER_CONSTANT
        );

        // 4) HSV + masks on flat image
        hsv = new Mat();
        Imgproc.cvtColor(topDown, hsv, Imgproc.COLOR_RGB2HSV);

        yellowMask = new Mat();
        Core.inRange(hsv, yellowLow, yellowHigh, yellowMask);
        mask = yellowMask.clone();

        if (Globals.ALLIANCE == Globals.COLORS.RED) {
            redMask1 = new Mat();
            redMask2 = new Mat();
            Core.inRange(hsv, redLow1, redHigh1, redMask1);
            Core.inRange(hsv, redLow2, redHigh2, redMask2);
            Core.bitwise_or(redMask1, redMask2, redMask1);
            Core.bitwise_or(mask, redMask1, mask);
        } else if (Globals.ALLIANCE == Globals.COLORS.BLUE) {
            blueMask = new Mat();
            Core.inRange(hsv, blueLow, blueHigh, blueMask);
            Core.bitwise_or(mask, blueMask, mask);
        }

        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        // 5) Apply ROI for search
        Rect roi = sanitizeRoi(new Rect(ROI_X, ROI_Y, ROI_WIDTH, ROI_HEIGHT), topDown.cols(), topDown.rows());
        Mat roiMask = new Mat(mask, roi);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(roiMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        foundBlock = false;
        double minDistance = Double.MAX_VALUE;
        RotatedRect closestRect = null;

        double centerX = imageWidth / 2.0;
        double centerY = imageHeight / 2.0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > 500) {
                // contour coords are relative to ROI; offset to global
                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
                rect.center.x += roi.x;
                rect.center.y += roi.y;

                double distance = Math.hypot(rect.center.x - centerX, rect.center.y - centerY);
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

            if (width < height) blockAngle += 90;
            if (blockAngle < 0) blockAngle += 180;

            dx = closestRect.center.x - centerX;
            dy = closestRect.center.y - centerY;
            foundBlock = true;

            // Determine color at center
            int cx = (int) Math.round(closestRect.center.x);
            int cy = (int) Math.round(closestRect.center.y);
            double r1 = redMask1 != null && !redMask1.empty() && inBounds(redMask1, cx, cy) ? redMask1.get(cy, cx)[0] : 0;
            double r2 = redMask2 != null && !redMask2.empty() && inBounds(redMask2, cx, cy) ? redMask2.get(cy, cx)[0] : 0;
            double yv = inBounds(yellowMask, cx, cy) ? yellowMask.get(cy, cx)[0] : 0;
            double bv = blueMask != null && !blueMask.empty() && inBounds(blueMask, cx, cy) ? blueMask.get(cy, cx)[0] : 0;

            if (r1 + r2 > 0)       blockColor = Globals.COLORS.RED;
            else if (yv > 0)       blockColor = Globals.COLORS.YELLOW;
            else if (bv > 0)       blockColor = Globals.COLORS.BLUE;
            else                   blockColor = Globals.COLORS.NONE;

            // Draw box
            Point[] box = new Point[4];
            closestRect.points(box);
            for (int i = 0; i < 4; i++) {
                Imgproc.line(topDown, box[i], box[(i + 1) % 4], new Scalar(0, 255, 0), 2);
            }

            Imgproc.putText(topDown,
                    String.format("Angle: %.1f deg dx: %.1f dy: %.1f clr: %s", blockAngle, dx, dy, blockColor),
                    closestRect.center, Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, new Scalar(255, 255, 0), 2);
        } else {
            blockColor = Globals.COLORS.NONE;
        }

        // cleanup masks
        if (redMask1 != null) redMask1.release();
        if (redMask2 != null) redMask2.release();
        if (yellowMask != null) yellowMask.release();
        if (blueMask != null) blueMask.release();
        if (roiMask != null) roiMask.release();

        // push final annotated flat image
        topDown.copyTo(input);

        // release temps
        if (undistorted != input) undistorted.release();
        if (hsv != null) hsv.release();
        if (mask != null) mask.release();
        topDown.release();

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
        distCoeffs.put(0, 0, Constants.k1, Constants.k2, Constants.p1, Constants.p2, Constants.k3);

        // initial dimensions
        imageWidth = WARP_WIDTH;
        imageHeight = WARP_HEIGHT;

        // default ROI to full frame
        ROI_X = 0; ROI_Y = 0; ROI_WIDTH = WARP_WIDTH; ROI_HEIGHT = WARP_HEIGHT;

        homographyDirty = true; // compute on first frame
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(0xFF00FF00);
        paint.setTextSize(50);

        if (foundBlock) {
            canvas.drawText(String.format("Angle: %.1f°  dx: %.1f  dy: %.1f", blockAngle, dx, dy), 40, 60, paint);
            canvas.drawText(String.format("Color: %s", blockColor), 40, 120, paint);
        } else {
            canvas.drawText("No block detected", 40, 60, paint);
        }

        if (SHOW_OVERLAY) {
            // draw ROI in green
            Paint roiPaint = new Paint();
            roiPaint.setStyle(Paint.Style.STROKE);
            roiPaint.setStrokeWidth(4.0f * scaleCanvasDensity);
            roiPaint.setColor(android.graphics.Color.GREEN);

            RectF roiScaled = new RectF(
                    ROI_X * scaleBmpPxToCanvasPx,
                    ROI_Y * scaleBmpPxToCanvasPx,
                    (ROI_X + ROI_WIDTH) * scaleBmpPxToCanvasPx,
                    (ROI_Y + ROI_HEIGHT) * scaleBmpPxToCanvasPx
            );
            canvas.drawRect(roiScaled, roiPaint);

            // help text
            Paint t = new Paint();
            t.setColor(0xFF00FFFF);
            t.setTextSize(22.0f * scaleCanvasDensity);
            canvas.drawText("Tune SRC_* for warp; ROI_* for search window", 40, 180, t);
        }
    }

    // ===== Helpers =====
    private static boolean inBounds(Mat m, int x, int y) {
        return x >= 0 && y >= 0 && x < m.cols() && y < m.rows();
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static Rect sanitizeRoi(Rect roi, int cols, int rows) {
        int x = clamp(roi.x, 0, cols);
        int y = clamp(roi.y, 0, rows);
        int w = clamp(roi.width, 1, cols - x);
        int h = clamp(roi.height, 1, rows - y);
        return new Rect(x, y, w, h);
    }

    // ===== Getters =====
    public double getBlockAngle() { return blockAngle; }
    public boolean isBlockFound() { return foundBlock; }
    public Globals.COLORS getBlockColor() { return blockColor; }
    public double getDx() { return dx; }
    public double getDy() { return dy; }

    // ===== Setters (for CameraSubsystem convenience) =====
    public void setWarpSize(int w, int h) {
        WARP_WIDTH = Math.max(32, w);
        WARP_HEIGHT = Math.max(32, h);
        homographyDirty = true;
    }

    public void setWarpCorners(double tlx, double tly,
                               double trx, double try_,
                               double brx, double bry,
                               double blx, double bly) {
        SRC_TL_X = tlx; SRC_TL_Y = tly;
        SRC_TR_X = trx; SRC_TR_Y = try_;
        SRC_BR_X = brx; SRC_BR_Y = bry;
        SRC_BL_X = blx; SRC_BL_Y = bly;
        homographyDirty = true;
    }

    public void setRoi(int x, int y, int w, int h) {
        ROI_X = x; ROI_Y = y; ROI_WIDTH = w; ROI_HEIGHT = h;
    }

    public void setUndistort(boolean enabled) { USE_UNDISTORT = enabled; }
    public void setOverlayEnabled(boolean show) { SHOW_OVERLAY = show; }
}
