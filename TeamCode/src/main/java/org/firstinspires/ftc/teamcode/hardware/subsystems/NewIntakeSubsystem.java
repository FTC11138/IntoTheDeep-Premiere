package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
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

    private final Servo arm1, arm2, intakePush;
    private final Servo intake;
    private final RevColorSensorV3 colorSensor;
    private Servo clawrotate;

    public RevBlinkinLedDriver leds;

    public SampleClawState sampleClawState;
    public SampleRotationState sampleRotationState;
    public ArmState armState;
    public IntakePushState intakePushState;
    public IntakeFlickState intakeFlickState;

    public enum IntakeFlickState {
        DOWN,
        READY,
    }
    public enum SampleRotationState {
        ROTATE,
        STOP,
    }
    public enum SampleClawState {
        OPEN,
        CLOSE,
    }

    public enum IntakePushState {
        PUSH,
        UP,
        STORE,
        DRIVE
    }

    public enum ArmState {
        TRANSFER,
        INTAKE,
        UP,
        FLAT,
        NONE
    }

    public NewIntakeSubsystem(HardwareMap hardwareMap, String ext, String arm1, String arm2, String intake, String leds, String colorSensor, String intakePush) {
        extensionParams = new RE_DcMotorExParams(
                Constants.extMin, Constants.extMax, Constants.extSlow,
                1, 1, Constants.extUpRatio, Constants.extDownRatio, Constants.extSlowRatio
        );
        this.extension = new RE_DcMotorEx(hardwareMap.get(DcMotorEx.class, ext), extensionParams);

        this.arm1 = hardwareMap.get(Servo.class, arm1);
        this.arm2 = hardwareMap.get(Servo.class, arm2);
        this.arm2.setDirection(Servo.Direction.REVERSE);

        this.intakePush = hardwareMap.get(Servo.class, intakePush);

        this.intake = hardwareMap.get(Servo.class, intake);
//        this.leds = hardwareMap.get(RevBlinkinLedDriver.class, leds);

        this.colorSensor = hardwareMap.get(RevColorSensorV3.class, colorSensor);

        sampleClawState = SampleClawState.OPEN;
        armState = ArmState.UP;
        intakePushState = IntakePushState.STORE;

        Robot.getInstance().subsystems.add(this);
    }

    @Override
    public void updateData() {
        Robot.getInstance().data.extensionPosition = this.extension.getPosition();
        Robot.getInstance().data.extensionTarget = this.extension.getTarget();
        Robot.getInstance().data.armStateNew = armState;
        Robot.getInstance().data.armPosition1 = arm1.getPosition();
        Robot.getInstance().data.armPosition2 = arm2.getPosition();
        Robot.getInstance().data.sampleClawState = sampleClawState;
    }

    public void setArmPosition(double pos) {
        this.arm1.setPosition(pos);
        this.arm2.setPosition(pos);
    }

    public void updateArmState(ArmState state) {
        this.armState = state;
    }

    public void updateIntakePushState(IntakePushState state) {
        this.intakePushState = state;
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
            default:
                return 0;
        }
    }

    private double getIntakePushStatePosition(IntakePushState state) {
        switch (state) {
            case PUSH:
                return Constants.intakePushDown;
            case UP:
                return Constants.intakePushUp;
            case STORE:
                return Constants.intakePushStore;
            case DRIVE:
                return Constants.intakePushDrive;
            default:
                return 0;
        }
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



    public double getDistance() {
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public void toggleLight() {
        colorSensor.enableLed(!colorSensor.isLightOn());
    }

    public Globals.COLORS getColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;

        if (red > 0.4 && green > 0.4 && blue < 0.2) {
            return Globals.COLORS.YELLOW;
        } else if (red > 0.5 && green < 0.3 && blue < 0.3) {
            return Globals.COLORS.RED;
        } else if (blue > 0.5 && red < 0.3 && green < 0.3) {
            return Globals.COLORS.BLUE;
        } else {
            return Globals.COLORS.NONE;
        }
    }



    @Override
    public void periodic() {
        this.colorSensor.setGain(Constants.colorSensorGain);

        this.extension.periodic();


        if (armState == ArmState.TRANSFER) {
            this.arm1.setPosition(getArmStatePosition(armState) - Constants.armServoOffsetTransfer);
        } else {
            this.arm1.setPosition(getArmStatePosition(armState) - Constants.armServoOffset);
        }
        this.arm2.setPosition(getArmStatePosition(armState));

        this.intakePush.setPosition(getIntakePushStatePosition(intakePushState));

        switch (this.sampleClawState) {
            case OPEN:
                intake.setPosition(1);
                break;
            case CLOSE:
                intake.setPosition(-1);
                break;
        }
    }

    public void setLeds(RevBlinkinLedDriver.BlinkinPattern pattern) {
        leds.setPattern(pattern);
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


}