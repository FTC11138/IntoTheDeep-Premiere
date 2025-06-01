package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Globals;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_DcMotorEx;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_DcMotorExParams;
import org.firstinspires.ftc.teamcode.util.wrappers.RE_SubsystemBase;

public class NewIntakeSubsystem extends RE_SubsystemBase {

    private final RE_DcMotorEx extension;
    private final RE_DcMotorExParams extensionParams;

    private final Servo arm1, arm2, claw, wrist, rotate;

    public RevBlinkinLedDriver leds;

    public ClawState clawState;
    public ArmState armState;
    public WristState wristState;
    public RotateState rotateState;
    public double clawAngle;

    // Constants for rotate servo angle calculation
    private final double ROTATE_TICKS_PER_DEGREE;

    public enum WristState {
        GRAB,
        STORE,
        TRANSFER,
        NONE
    }

    public enum ClawState {
        OPEN,
        CLOSE,
        NONE
    }

    public enum RotateState {
        HORIZONTAL,
        VERTICAL,
        NONE
    }

    public enum ArmState {
        TRANSFER,
        INTAKE,
        UP,
        DOWN,
        FLAT,
        NONE
    }

    public enum NewIntakePushState {
        PUSH,
        UP,
        STORE,
        DRIVE,
        NONE
    }

    public NewIntakeSubsystem(HardwareMap hardwareMap, String ext, String arm1, String arm2, String claw, String rotate, String wrist) {
        extensionParams = new RE_DcMotorExParams(
                Constants.extMin, Constants.extMax, Constants.extSlow,
                1, 1, Constants.extUpRatio, Constants.extDownRatio, Constants.extSlowRatio
        );
        this.extension = new RE_DcMotorEx(hardwareMap.get(DcMotorEx.class, ext), extensionParams);

        this.arm1 = hardwareMap.get(Servo.class, arm1);
        this.arm2 = hardwareMap.get(Servo.class, arm2);
        this.arm2.setDirection(Servo.Direction.REVERSE);

        this.claw = hardwareMap.get(Servo.class, claw);
        this.wrist = hardwareMap.get(Servo.class, wrist);
        this.rotate = hardwareMap.get(Servo.class, rotate);

        // Calculate degrees per servo unit for rotation (0-1 corresponds to 0-180 degrees)
        ROTATE_TICKS_PER_DEGREE = (Constants.rotate180 - Constants.rotate0) / 180.0;

        clawState = ClawState.OPEN;
        wristState = WristState.STORE;
        armState = ArmState.UP;
        rotateState = RotateState.NONE;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.extensionPosition = this.extension.getPosition();
        Robot.getInstance().data.extensionTarget = this.extension.getTarget();
        Robot.getInstance().data.armStateNew = armState;
        Robot.getInstance().data.armPosition1 = arm1.getPosition();
        Robot.getInstance().data.armPosition2 = arm2.getPosition();
        Robot.getInstance().data.clawState = clawState;
        Robot.getInstance().data.wristState = wristState;
        Robot.getInstance().data.clawAngle = clawAngle;
    }

    public void setArmPosition(double pos) {
        this.arm1.setPosition(pos);
        this.arm2.setPosition(pos);
    }

    public void updateArmState(ArmState state) {
        this.armState = state;
    }

    public void updateWristState(WristState state) {
        this.wristState = state;
    }

    public void updateClawState(ClawState state) {
        this.clawState = state;
    }

    public void updateRotateState(RotateState state) {
        this.rotateState = state;
    }


    private double getArmStatePosition(ArmState state) {
        switch (state) {
            case TRANSFER:
                return Constants.armTransfer;
            case INTAKE:
                return Constants.armIntake;
            case UP:
                return Constants.armUp;
            case FLAT:
                return Constants.armFlat;
            case DOWN:
                return Constants.armDown;
            default:
                return 0;
        }
    }

    private double getWristStatePosition(WristState state) {
        switch (state) {
            case GRAB:
                return Constants.wristGrab;
            case STORE:
                return Constants.wristStore;
            case TRANSFER:
                return Constants.wristTransfer;
            default:
                return Constants.wristStore;
        }
    }

    private double getClawStatePosition(ClawState state) {
        switch (state) {
            case OPEN:
                return Constants.clawOpen;
            case CLOSE:
                return Constants.clawClose;
            default:
                return Constants.clawOpen;
        }
    }

    private double getRotateStatePosition(RotateState state) {
        switch (state) {
            case HORIZONTAL:
                return Constants.rotateHorizontal;
            case VERTICAL:
                return Constants.rotateVertical;
            case NONE:
                return clawAngle;
            default:
                return Constants.rotateHorizontal;
        }
    }

    public void setClawPosition(double position) {
        this.claw.setPosition(position);
    }

    public void setWristPosition(double position) {
        this.wrist.setPosition(position);
    }

    public void setRotatePosition(double position) {
        this.rotate.setPosition(position);
    }

    public void setRotateAngle(double angle) {
        angle = normalizeAngle(angle);

        double servoPosition = angle * ROTATE_TICKS_PER_DEGREE;
        setRotatePosition(servoPosition);

        this.rotateState = RotateState.NONE;
        this.clawAngle = angle;
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle < 0) {
            angle += 360;
        }

        if (angle > 180) {
            angle = 360 - angle;
        }

        return angle;
    }

    public void setExtensionPower(double power) {
        this.extension.setPower(power);
    }

    public void setTargetExtensionPosition(int target) {
        this.extension.setTargetPosition(target, this.extensionParams.maxPower);
    }

    public void setExtensionPosition(double power, int target) {
        this.extension.setPosition(power, target);
    }

    @Override
    public void periodic() {
        this.extension.periodic();

        if (armState == ArmState.TRANSFER) {
            this.arm1.setPosition(getArmStatePosition(armState) - Constants.armServoOffsetTransfer);
        } else {
            this.arm1.setPosition(getArmStatePosition(armState) - Constants.armServoOffset);
        }
        this.arm2.setPosition(getArmStatePosition(armState));

        // Update the claw, wrist, and rotate positions based on their states
        this.claw.setPosition(getClawStatePosition(clawState));
        this.wrist.setPosition(getWristStatePosition(wristState));

        setRotateAngle(getRotateStatePosition(rotateState));
    }

    public void resetExtension() {
        this.extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getExtensionPosition() {
        return extension.getPosition();
    }

    public boolean extensionBusy() {
        return extension.motor.isBusy();
    }


    public double getClawPosition() {
        return claw.getPosition();
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }

    public double getRotatePosition() {
        return rotate.getPosition();
    }

    public double getRotateAngle() {
        return clawAngle;
    }
}