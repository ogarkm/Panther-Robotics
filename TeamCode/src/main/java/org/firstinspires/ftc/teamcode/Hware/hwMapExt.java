package org.firstinspires.ftc.teamcode.Hware;

import static org.firstinspires.ftc.teamcode.teleOp.Constants.DriveConstants;
import static org.firstinspires.ftc.teamcode.teleOp.Constants.TransferConstants;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import android.graphics.Color;


public class hwMapExt {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;


    public Servo flickA;
    public Servo flickB;
    public Servo flickC;

    public NormalizedColorSensor indexA;
    public NormalizedColorSensor indexB;
    public NormalizedColorSensor indexC;

    public Servo[] lifts;
    public NormalizedColorSensor[] sensors;




    /*   Indexer Positions (technically not an indexer I know :/ )
         ____
        |_ 2_|
       /  \/  \
      /_1_/\_3_\
         Front

      1 = A, 2 = B, 3 = C; If u needed this, ur IQ ain't above 99... js saying
     */

    public hwMapExt(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.dcMotor.get(DriveConstants.FRONT_LEFT_MOTOR);
        backLeftMotor = hardwareMap.dcMotor.get(DriveConstants.BACK_LEFT_MOTOR);
        frontRightMotor = hardwareMap.dcMotor.get(DriveConstants.FRONT_RIGHT_MOTOR);
        backRightMotor = hardwareMap.dcMotor.get(DriveConstants.BACK_RIGHT_MOTOR);

        frontLeftMotor.setDirection(DriveConstants.FRONT_LEFT_DIRECTION);
        backLeftMotor.setDirection(DriveConstants.BACK_LEFT_DIRECTION);
        frontRightMotor.setDirection(DriveConstants.FRONT_RIGHT_DIRECTION);
        backRightMotor.setDirection(DriveConstants.BACK_RIGHT_DIRECTION);

        setMotorZero(DcMotor.ZeroPowerBehavior.BRAKE);
        setMotorModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //transfer system start
        flickA = hardwareMap.servo.get(TransferConstants.LIFT_SERVO_A);
        flickB = hardwareMap.servo.get(TransferConstants.LIFT_SERVO_B);
        flickC = hardwareMap.servo.get(TransferConstants.LIFT_SERVO_C);

        indexA = hardwareMap.get(NormalizedColorSensor.class, TransferConstants.INDEX_SENSOR_A);
        indexB = hardwareMap.get(NormalizedColorSensor.class, TransferConstants.INDEX_SENSOR_B);
        indexC = hardwareMap.get(NormalizedColorSensor.class, TransferConstants.INDEX_SENSOR_C);

        NormalizedColorSensor[] sensors = { indexA, indexB, indexC };
        Servo[] lifts = {flickA, flickB, flickC};

        for (NormalizedColorSensor sensor : sensors) {
            if (sensor instanceof SwitchableLight) {
                ((SwitchableLight) sensor).enableLight(true);
            }
        }
    }

    public void checkFlickPos() {
        for (Servo servo : lifts) {
            if (servo.getPosition() == TransferConstants.FLICK_POS_UP) {
                servo.setPosition(TransferConstants.FLICK_POS_DOWN);
            }
        }
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        frontLeftMotor.setMode(mode);
        backLeftMotor.setMode(mode);
        frontRightMotor.setMode(mode);
        backRightMotor.setMode(mode);
    }


    public void setMotorZero(DcMotor.ZeroPowerBehavior zero) {
        frontLeftMotor.setZeroPowerBehavior(zero);
        backLeftMotor.setZeroPowerBehavior(zero);
        frontRightMotor.setZeroPowerBehavior(zero);
        backRightMotor.setZeroPowerBehavior(zero);
    }

    public void setMotorPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void resetEncoders() {
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int[] getEncoderPositions() {
        return new int[] {
                frontLeftMotor.getCurrentPosition(),
                frontRightMotor.getCurrentPosition(),
                backLeftMotor.getCurrentPosition(),
                backRightMotor.getCurrentPosition()
        };
    }



    public void setTransferPos(int index, boolean up) { // index takes 1, 2, 3

        lifts[index - 1].setPosition(
                up ? TransferConstants.FLICK_POS_UP : TransferConstants.FLICK_POS_DOWN
        );
    }


    public int detectArtifactColor(int index) {
        // Read color
        NormalizedRGBA colors = sensors[index-1].getNormalizedColors();

        // Convert to HSV
        float[] hsv = new float[3];
        Color.colorToHSV(colors.toColor(), hsv);

        float hue = hsv[0];
        float sat = hsv[1];
        float val = hsv[2];
        if (sat < 0.2 || val < 0.1) {
            return 0; // unable to view
        }

        // PURPLE RANGE
        if (hue > 260 && hue < 300) {
            return 1;
        }

        // GREEN RANGE
        if (hue > 80 && hue < 160) {
            return 2;
        }

        return 0; // could be error, or could be no artifact... *sigh*
    }


}