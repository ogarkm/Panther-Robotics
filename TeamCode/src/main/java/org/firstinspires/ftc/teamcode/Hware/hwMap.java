package org.firstinspires.ftc.teamcode.Hware;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teleOp.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.configuration.annotations.ServoTypes;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import org.firstinspires.ftc.teamcode.teleOp.Constants;

public class hwMap {
    public static class IntakeHwMap {
        public DcMotor frontIntakeMotor;
        public DcMotor backIntakeMotor;

        public IntakeHwMap(HardwareMap hardwareMap) {
            frontIntakeMotor = hardwareMap.dcMotor.get(Constants.IntakeConstants.FRONT_INTAKE_MOTOR);
            backIntakeMotor = hardwareMap.dcMotor.get(Constants.IntakeConstants.BACK_INTAKE_MOTOR);

            frontIntakeMotor.setDirection(Constants.IntakeConstants.FRONT_INTAKE_DIRECTION);
            backIntakeMotor.setDirection(Constants.IntakeConstants.BACK_INTAKE_DIRECTION);

            frontIntakeMotor.setZeroPowerBehavior(Constants.IntakeConstants.INTAKE_ZERO_POWER_BEHAVIOR);
            backIntakeMotor.setZeroPowerBehavior(Constants.IntakeConstants.INTAKE_ZERO_POWER_BEHAVIOR);

            frontIntakeMotor.setMode(Constants.IntakeConstants.INTAKE_RUNMODE);
            backIntakeMotor.setMode(Constants.IntakeConstants.INTAKE_RUNMODE);
        }

    }
    // public static class LiftHwMap {
     //   public Servo ptoLeft;
       // public Servo ptoRight;
        //public LiftHwMap(HardwareMap hardwareMap) {
           // ptoLeft = hardwareMap.servo.get(Constants.LiftConstants.PTO_LEFT);
           // ptoRight = hardwareMap.servo.get(Constants.LiftConstants.PTO_RIGHT);

           // ptoRight.setDirection(Constants.LiftConstants.CW);
         //   ptoLeft.setDirection(Constants.LiftConstants.CCW);
        // }

    // }
    public static class DriveHwMap {
        public DcMotor frontLeftMotor;
        public DcMotor backLeftMotor;
        public DcMotor frontRightMotor;
        public DcMotor backRightMotor;
        public DriveHwMap(HardwareMap hardwareMap) {
            frontLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_LEFT_MOTOR);
            backLeftMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_LEFT_MOTOR);
            frontRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.FRONT_RIGHT_MOTOR);
            backRightMotor = hardwareMap.dcMotor.get(Constants.DriveConstants.BACK_RIGHT_MOTOR);

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            setDriveMotorZero(DcMotor.ZeroPowerBehavior.BRAKE);
            setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void setDriveMotorZero(DcMotor.ZeroPowerBehavior zero) {
            frontLeftMotor.setZeroPowerBehavior(zero);
            backLeftMotor.setZeroPowerBehavior(zero);
            frontRightMotor.setZeroPowerBehavior(zero);
            backRightMotor.setZeroPowerBehavior(zero);
        }
        public void setMotorTargetPositions(double inches){
            setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setTargetPosition(-inchesToTicks(inches));
            frontRightMotor.setTargetPosition(inchesToTicks(inches));
            backLeftMotor.setTargetPosition(-inchesToTicks(inches));
            backRightMotor.setTargetPosition(inchesToTicks(inches));

        }
        public void setDriveMotorModes(DcMotor.RunMode mode) {
            frontLeftMotor.setMode(mode);
            backLeftMotor.setMode(mode);
            frontRightMotor.setMode(mode);
            backRightMotor.setMode(mode);
        }

        public void setMotorPowers(double frontLeftPower, double frontRightPower,
                                   double backLeftPower, double backRightPower) {
            frontLeftMotor.setPower(frontLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backLeftMotor.setPower(backLeftPower);
            backRightMotor.setPower(backRightPower);
        }

        public void setMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior mode) {
            frontLeftMotor.setZeroPowerBehavior(mode);
            backLeftMotor.setZeroPowerBehavior(mode);
            frontRightMotor.setZeroPowerBehavior(mode);
            backRightMotor.setZeroPowerBehavior(mode);
        }

        public void stopMotors() {
            setMotorPowers(0, 0, 0, 0);
        }

        public void resetEncoders() {
            setDriveMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            setDriveMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public int[] getEncoderPositions() {
            return new int[] {
                    frontLeftMotor.getCurrentPosition(),
                    frontRightMotor.getCurrentPosition(),
                    backLeftMotor.getCurrentPosition(),
                    backRightMotor.getCurrentPosition()
            };
        }

        private int inchesToTicks(double inches) {
            double wheelDiameter = Constants.DriveConstants.WHEEL_DIAMETER;  // in inches
            double ticksPerRev = Constants.DriveConstants.TICKS_PER_REVOLUTION;
            double gearRatio = 2.0;
            double circumference = Math.PI * wheelDiameter;

            return (int) (inches * (ticksPerRev * gearRatio) / circumference);
        }
    }

    public static class TransferHwMap {
        public Servo flickA;
        public Servo flickB;
        public Servo flickC;

        public NormalizedColorSensor indexA;
        public NormalizedColorSensor indexB;
        public NormalizedColorSensor indexC;

        // Class fields
        public Servo[] lifts;
        public NormalizedColorSensor[] sensors;

        public TransferHwMap(HardwareMap hardwareMap) {
            flickA = hardwareMap.servo.get(Constants.TransferConstants.LIFT_SERVO_A);
            flickB = hardwareMap.servo.get(Constants.TransferConstants.LIFT_SERVO_B);
            flickC = hardwareMap.servo.get(Constants.TransferConstants.LIFT_SERVO_C);

            indexA = hardwareMap.get(NormalizedColorSensor.class, Constants.TransferConstants.INDEX_SENSOR_A);
            indexB = hardwareMap.get(NormalizedColorSensor.class, Constants.TransferConstants.INDEX_SENSOR_B);
            indexC = hardwareMap.get(NormalizedColorSensor.class, Constants.TransferConstants.INDEX_SENSOR_C);

            this.sensors = new NormalizedColorSensor[]{ indexA, indexB, indexC };
            this.lifts = new Servo[]{flickA, flickB, flickC};

            for (NormalizedColorSensor sensor : sensors) {
                if (sensor instanceof SwitchableLight) {
                    ((SwitchableLight) sensor).enableLight(true);
                }
            }
        }

        public void setTransferPos(int index, boolean up) { // index takes 1, 2, 3
            if(index < 1 || index > 3) return;
            lifts[index - 1].setPosition(
                    up ? Constants.TransferConstants.FLICK_POS_UP : Constants.TransferConstants.FLICK_POS_DOWN
            );
        }

        public int detectArtifactColor(int index) {
            if(index < 1 || index > 3) return 0;
            NormalizedRGBA colors = sensors[index-1].getNormalizedColors();

            float[] hsv = new float[3];
            Color.colorToHSV(colors.toColor(), hsv);
            float hue = hsv[0];
            float sat = hsv[1];
            float val = hsv[2];

            if (sat < 0.2 || val < 0.1) return 0;
            if (hue > 260 && hue < 300) return 1; // Purple
            if (hue > 80 && hue < 160) return 2;  // Green

            return 0;
        }
    }

    public static class TurretHwMap {

        private static final boolean USE_WEBCAM = true;
        public AprilTagProcessor aprilTag;
        public VisionPortal visionPortal;


        public DcMotor turretLeftMotor;
        public DcMotor turretRightMotor;

        public CRServo turretservo;
        public Servo hoodservo;

        public TurretHwMap(HardwareMap hardwareMap) {
            turretLeftMotor = hardwareMap.dcMotor.get(Constants.TurretConstants.TURRET_LEFT_MOTOR);
            turretRightMotor = hardwareMap.dcMotor.get(Constants.TurretConstants.TURRET_RIGHT_MOTOR);

            turretLeftMotor.setDirection(Constants.TurretConstants.LEFT_TURRET_MOTOR_DIRECTION);
            turretRightMotor.setDirection(Constants.TurretConstants.RIGHT_TURRET_MOTOR_DIRECTION);

            turretservo = hardwareMap.crservo.get(Constants.TurretConstants.LEFT_TURRET_SERVO);
            hoodservo = hardwareMap.servo.get(Constants.TurretConstants.HOOD_TURRET_SERVO);

            initAprilTag(hardwareMap);
        }

        public void setTurretPower(double power) {
            turretLeftMotor.setPower(power);
            turretRightMotor.setPower(power);
        }

        public void turretOff() {
            turretLeftMotor.setPower(0);
            turretRightMotor.setPower(0);
        }

        public void setHoodPos(double pos) {
            hoodservo.setPosition(pos);
        }

        public void setTurretRotationPower(double power) {
            turretservo.setPower(power);
        }

        public void stopTurretRotation() {
            turretservo.setPower(0);
        }
        public AprilTagDetection getAprilTagById(int targetTagId) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null && detection.id == targetTagId) {
                    return detection;
                }
            }

            return null;  // Not found
        }

        public void initAprilTag(HardwareMap hardwareMap) {
            // Create the AprilTag processor.
            aprilTag = AprilTagProcessor.easyCreateWithDefaults();

            // Create the vision portal.
            if (USE_WEBCAM) {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
            } else {
                visionPortal = VisionPortal.easyCreateWithDefaults(
                        BuiltinCameraDirection.BACK, aprilTag);
            }
        }
    }

}