package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;
import org.firstinspires.ftc.teamcode.vision.SampleAngleProcessor;
import org.firstinspires.ftc.teamcode.vision.NewTestingSampleAngleProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
public class NewTestCameraSubsystem extends RE_SubsystemBase {

    // ===== Vision stack =====
    private WebcamName intakeCamera;
    private VisionPortal visionPortal;
    private NewTestingSampleAngleProcessor sampleAngleProcessor;

    public enum CameraState { ON, OFF }
    private CameraState cameraState = CameraState.OFF;

    // ===== Vision inputs (from SampleAngleProcessor) =====
    private double sampleDxPx = 0.0;   // pixels (x to the right in flat image)
    private double sampleDyPx = 0.0;   // pixels (y down in flat image)
    private boolean targetSeen = false;
    private double targetAngleDeg = 0.0;

    // ===== Pixel → meter scaling for the bird’s-eye image (tune in Dashboard) =====
    public static double M_PER_PX_X = 0.001;  // meters per pixel along flat image X
    public static double M_PER_PX_Y = 0.001;  // meters per pixel along flat image Y
    // If dx/dy are measured from the image center (they are in the provided processor), leave 0:
    public static double ORIGIN_X_PX = 0.0;
    public static double ORIGIN_Y_PX = 0.0;

    // ===== Field (flat image) → Robot transform (rotate then translate) =====
    public static double FIELD_TO_ROBOT_THETA_DEG = 0.0; // rotation of flat image frame to robot base frame
    public static double FIELD_TO_ROBOT_TX_M = 0.0;      // translation X (m)
    public static double FIELD_TO_ROBOT_TY_M = 0.0;      // translation Y (m)

    // ===== 2-link planar arm (shoulder + "wrist" as second joint) =====
    public static double L1_M = 0.22;  // shoulder→elbow (m)
    public static double L2_M = 0.18;  // elbow→wrist (your “wrist link”) (m)

    // Distance from wrist joint to actual grasp point (TCP) along tool axis
    public static double TCP_OFFSET_M = 0.05;

    // Slide/prismatic axis: target pick height relative to arm base
    public static double Z_PICKUP_M = 0.05;
    public static double Z_BASE_M   = 0.00;
    public static double SLIDE_MIN_M = 0.00, SLIDE_MAX_M = 0.60;

    // Joint limits (radians)
    public static double SHOULDER_MIN_RAD = Math.toRadians(-120);
    public static double SHOULDER_MAX_RAD = Math.toRadians( 120);
    public static double WRIST_MIN_RAD    = Math.toRadians(-140);
    public static double WRIST_MAX_RAD    = Math.toRadians( 140);

    // Prefer elbow-up branch first (if infeasible, fallback to elbow-down)
    public static boolean ELBOW_UP = true;

    // ===== IK outputs (public getters below) =====
    private boolean ikValid = false;
    private double targetX_m = 0.0;        // robot-frame target (from vision)
    private double targetY_m = 0.0;
    private double targetYaw_rad = 0.0;    // from block angle (optional use)

    private double targetShoulder_rad = 0.0; // joint 1
    private double targetWrist_rad    = 0.0; // joint 2 (your “wrist”)
    private double targetSlide_m      = 0.0; // prismatic slide extension

    public NewTestCameraSubsystem(HardwareMap hardwareMap, String intakeCameraName) {
        this.intakeCamera = hardwareMap.get(WebcamName.class, intakeCameraName);
        setupCamera();
        cameraState = CameraState.ON;
        Robot.getInstance().subsystems.add(this);
    }

    private void setupCamera() {
        // IMPORTANT: this uses the SampleAngleProcessor I gave you earlier
        // (with Core.BORDER_CONSTANT, Calib3d.undistort, bird’s-eye warp, ROI, overlays)
        sampleAngleProcessor = new NewTestingSampleAngleProcessor();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(sampleAngleProcessor)
                .setCamera(intakeCamera)
                .build();
    }

    public void updateCameraState(CameraState state) {
        cameraState = state;
        if (state == CameraState.ON) visionPortal.resumeStreaming();
        else                         visionPortal.stopStreaming();
    }

    @Override
    public void periodic() {
        if (cameraState != CameraState.ON) {
            ikValid = false;
            return;
        }

        // 1) Read vision from the SampleAngleProcessor (provided version)
        targetSeen     = sampleAngleProcessor.isBlockFound();
        sampleDxPx     = sampleAngleProcessor.getDx();          // pixels (center-relative)
        sampleDyPx     = sampleAngleProcessor.getDy();          // pixels (center-relative)
        targetAngleDeg = sampleAngleProcessor.getBlockAngle();  // degrees

        if (!targetSeen) {
            ikValid = false;
            return;
        }

        // 2) Pixels → field (flat image) meters
        double x_field_m = (ORIGIN_X_PX + sampleDxPx) * M_PER_PX_X;
        double y_field_m = (ORIGIN_Y_PX + sampleDyPx) * M_PER_PX_Y;

        // 3) Field → robot base frame (rotate then translate)
        double th = Math.toRadians(FIELD_TO_ROBOT_THETA_DEG);
        double c = Math.cos(th), s = Math.sin(th);
        double xr =  c * x_field_m - s * y_field_m + FIELD_TO_ROBOT_TX_M;
        double yr =  s * x_field_m + c * y_field_m + FIELD_TO_ROBOT_TY_M;

        // 4) Desired gripper yaw from vision (radians)
        double psi = Math.toRadians(targetAngleDeg);

        // Save target pose (pre-IK)
        targetX_m = xr;
        targetY_m = yr;
        targetYaw_rad = psi;

        // 5) Account for TCP offset: move target back along tool axis to wrist joint
        double xrAdj = xr - TCP_OFFSET_M * Math.cos(psi);
        double yrAdj = yr - TCP_OFFSET_M * Math.sin(psi);

        // 6) Solve 2-link planar IK (shoulder + wrist)
        IK2Link.Solution up = IK2Link.solve(L1_M, L2_M, xrAdj, yrAdj, psi, true,
                SHOULDER_MIN_RAD, SHOULDER_MAX_RAD, WRIST_MIN_RAD, WRIST_MAX_RAD);
        IK2Link.Solution down = IK2Link.solve(L1_M, L2_M, xrAdj, yrAdj, psi, false,
                SHOULDER_MIN_RAD, SHOULDER_MAX_RAD, WRIST_MIN_RAD, WRIST_MAX_RAD);

        IK2Link.Solution pick = ELBOW_UP ? up : down;
        if (!pick.feasible) pick = ELBOW_UP ? down : up;

        ikValid = pick.feasible;

        if (ikValid) {
            targetShoulder_rad = pick.shoulder;
            targetWrist_rad    = pick.wrist;

            // 7) Slide height
            double sExt = clamp(Z_PICKUP_M - Z_BASE_M, SLIDE_MIN_M, SLIDE_MAX_M);
            targetSlide_m = sExt;
        }
    }

    // ===== Tiny 2-link IK helper (shoulder + wrist) =====
    private static final class IK2Link {
        static final class Solution {
            final double shoulder, wrist; final boolean feasible;
            Solution(double s, double w, boolean ok) { shoulder=s; wrist=w; feasible=ok; }
        }
        // lengths (m), angles (rad); elbowUp picks the positive sqrt branch
        static Solution solve(double L1, double L2, double x, double y, double desiredYaw,
                              boolean elbowUp,
                              double shoulderMin, double shoulderMax,
                              double wristMin, double wristMax) {
            double r2 = x*x + y*y;
            double c2 = (r2 - L1*L1 - L2*L2) / (2.0*L1*L2);
            c2 = clamp(c2, -1.0, 1.0);
            double s2 = Math.sqrt(Math.max(0.0, 1.0 - c2*c2));
            if (!elbowUp) s2 = -s2;

            double theta2 = Math.atan2(s2, c2); // wrist joint (relative angle between links)
            double k1 = L1 + L2 * Math.cos(theta2);
            double k2 = L2 * Math.sin(theta2);
            double theta1 = Math.atan2(y, x) - Math.atan2(k2, k1); // shoulder
            double thetaSum = theta1 + theta2;
            double thetaWrist = desiredYaw - thetaSum; // adjust so end-effector yaw = desiredYaw

            theta1 = normalize(theta1);
            thetaWrist = normalize(thetaWrist);

            boolean ok = inRange(theta1, shoulderMin, shoulderMax) && inRange(thetaWrist, wristMin, wristMax);
            return new Solution(theta1, thetaWrist, ok);
        }
        private static boolean inRange(double a, double lo, double hi) { return a >= lo && a <= hi; }
        private static double normalize(double a) {
            while (a > Math.PI) a -= 2*Math.PI;
            while (a < -Math.PI) a += 2*Math.PI;
            return a;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ===== Public getters for your arm/slide controllers =====
    public CameraState getCameraState()        { return cameraState; }
    public boolean isTargetSeen()              { return targetSeen; }
    public boolean isIkValid()                 { return ikValid; }
    public double getTargetX_m()               { return targetX_m; }
    public double getTargetY_m()               { return targetY_m; }
    public double getTargetYaw_rad()           { return targetYaw_rad; }
    public double getTargetShoulder_rad()      { return targetShoulder_rad; }
    public double getTargetWrist_rad()         { return targetWrist_rad; }
    public double getTargetSlide_m()           { return targetSlide_m; }

    // (Optional) quick access to vision raw values
    public double getVisionDxPx()              { return sampleDxPx; }
    public double getVisionDyPx()              { return sampleDyPx; }
    public double getVisionAngleDeg()          { return targetAngleDeg; }
}