package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Config
public class Constants {

    public static RevBlinkinLedDriver.BlinkinPattern closePattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    public static RevBlinkinLedDriver.BlinkinPattern openPattern = RevBlinkinLedDriver.BlinkinPattern.RED;

    public static RevBlinkinLedDriver.BlinkinPattern redPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static RevBlinkinLedDriver.BlinkinPattern whitePattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
    public static RevBlinkinLedDriver.BlinkinPattern bluePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public static RevBlinkinLedDriver.BlinkinPattern greenPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;

    /* -------------------------------------------- TELE OP CONSTANTS -------------------------------------------- */


    public static float colorSensorGain = 2;


    /* -------------------------------------------- AUTO CONSTANTS -------------------------------------------- */

    public static double samplePickupTolerance = 1.8;
    public static double samplePickupTurnSpeedTolerance = 5000;

    /* -------------------------------------------- DRIVE CONSTANTS -------------------------------------------- */


    public static double pathEndXTolerance = 1;
    public static double pathEndYTolerance = 1;
    public static double pathEndHeadingTolerance = Math.toRadians(2);

    public static boolean robotCentric = false;


    /* -------------------------------------------- SERVO CONSTANTS -------------------------------------------- */

    public static double bucketDrop = 0.7;
    public static double bucketIntake = 0.95;
    public static double bucketAscent = 0;

    public static double armTransfer = 0.87;
    public static double armIntake = 0.56;
    public static double armDown = 0.54;
    public static double armDownSub = 0.52;
    public static double armAuto = 0.58;
    public static double armFlat = 0.68;
    public static double armUp = 0.76;
    public static double armServoOffset = -0.055;
    public static double armServoOffsetTransfer = -0.02;

    public static double intakePushDown = 0.35;
    public static double intakePushUp = 0.7;
    public static double intakePushStore = 0.57;
    public static double intakePushDrive = 0.1;

    public static double specimenClawOpen = 0.1;
    public static double specimenClawClose = 0.67;

    public static double clawOpen = 0.4;
    public static double clawOpenWide = 0.25;
    public static double clawClose = 0.6;

    public static double wristGrab = 0.05;
    public static double wristPreGrabForward = 0.1;
    public static double wristPreGrabBack = 0.23;
    public static double wristPreGrab = 0.19;
    public static double wristStore = 0.3;
    public static double wristTransfer = 0.74;
    public static double wristAutoPickup = 0;

    public static double rotateHorizontal = 0;
    public static double rotateVertical = 90;
    public static double rotate0 = 0.3;
    public static double rotate180 = 0.97;


    /* -------------------------------------------- MOTOR CONSTANTS -------------------------------------------- */

    // LIFT
    public static int liftMin = 0;
    public static int liftMax = 1300;
    public static int liftMid = 700;
    public static int liftSlow = 400;
    public static int liftAscent = 470;
    public static int liftOffset = 200;

    public static double liftUpRatio = 1;
    public static double liftDownRatio = 1;
    public static double liftSlowRatio = .4;

    // Extension
    public static int extMin = 0;
    public static int extIntake = 400;
    public static int extMax = 1500;
    public static int extSlow = 50;

    public static int extJump = 500;
    public static int extGrabFowardOffset = 200;
    public static int extGrabBackOffset = 200;

    public static double extUpRatio = 1;
    public static double extDownRatio = 1;
    public static double extSlowRatio = 1;

    //
    public static int specimenLiftMin = 0;
    public static int specimenLiftLow = 100;
    public static int specimenLiftHigh = 270;
    public static int specimenLiftMax = 280;
    public static int specimenLiftSlow = 50;

    public static int specimenLiftGrab = 0;
    public static int specimenLiftGrabbedOffset = 100;
    public static int specimenLiftHangOffset = 300;
    public static int specimenLiftHangTime = 250;
    public static double specimenLiftHangPower = 1;

    public static double specimenLiftUpRatio = 1;
    public static double specimenLiftDownRatio = 1;
    public static double specimenLiftSlowRatio = 1;

    public static double specimenLiftMaxPower = 1;



    /* -------------------------------------------- VISION RECTANGLE CONSTANTS -------------------------------------------- */

    public static double kPAngle = -0.005;
    public static double kIAngle = 0;
    public static double kDAngle = 0;
    public static double kFAngle = 0;

    public static double sampleAlignAngleTolerance = 2;

    public static int sampleDyCorrectionConstantFarBehind = 0;
    public static int sampleDyCorrectionConstantFarForward = 0;
    public static int sampleDyCorrectionConstantBehind = -320;
    public static int sampleDyCorrectionConstantForward = -150;

    public static int sampleDyMiddleCorrectionConstant = 60;
    public static int sampleDyForwardFarCorrectionConstant = -90;
    public static int sampleDyBackwardFarCorrectionConstant = 42;

//    public static double sampleDyCorrectionMultiplier1 = -1.48371;
//    public static double sampleDyCorrectionMultiplier2 = 0.00344716;
//    public static double sampleDyCorrectionMultiplier3 = -0.0000240512;
//    public static double sampleDyCorrectionMultiplier4 = 0.0000001;

    public static double sampleDyCorrectionMultiplier1 = -1.5;
    public static double sampleDyCorrectionMultiplier2 = 0;
    public static double sampleDyCorrectionMultiplier3 = 0;
    public static double sampleDyCorrectionMultiplier4 = 0;


    public static double sampleDxCorrectionMultiplier = -0.02;
    public static int sampleDxConstant = -20;

    public static double fX = 203.7;
    public static double fY = 203.7;
    public static double cX = 320;
    public static double cY = 240;

    public static double k1 = -0.38;
    public static double k2 = 0.15;
    public static double p1 = 0;
    public static double p2 = 0;
    public static double k3 = 0;





    /* --------------------------------------------  APRILTAG CONSTANTS ------------------------------------------ */

    public static double monkey = 8;

}
