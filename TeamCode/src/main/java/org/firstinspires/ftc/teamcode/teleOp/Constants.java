package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Constants {

    // Drive Constants
    public static class DriveConstants {
        public static final String FRONT_LEFT_MOTOR = "fl";
        public static final String FRONT_RIGHT_MOTOR = "fr";
        public static final String BACK_LEFT_MOTOR = "bl";
        public static final String BACK_RIGHT_MOTOR = "br";
        public static final DcMotorSimple.Direction FRONT_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction FRONT_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_LEFT_DIRECTION = DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction BACK_RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final double TRACK_WIDTH = 13.5;
        public static final double WHEEL_BASE = 13.5;
        public static final double WHEEL_DIAMETER = 4.0;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        public static final double TICKS_PER_REVOLUTION = 537.6;

        // Drive characteristics
        public static final double MAX_VELOCITY = 30.0; // inches per second
        public static final double MAX_ANGULAR_VELOCITY = Math.toRadians(180.0); // radians per second
        public static final double MAX_ACCELERATION = 30.0; // inches per second squared

        // PID coefficients for autonomous driving
        public static final double DRIVE_kP = 0.1;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

        public static final double TURN_kP = 0.1;
        public static final double TURN_kI = 0.0;
        public static final double TURN_kD = 0.0;

        public static final double STOP_SPEED_MULTIPLIER = 0.0;
        public static final double PRECISION_SPEED_MULTIPLIER = 0.3;
        public static final double NORMAL_SPEED_MULTIPLIER = 0.6;
        public static final double TURBO_SPEED_MULTIPLIER = 1.0;
    }

    public static class IntakeConstants {
        public static final String FRONT_INTAKE_MOTOR = "frontIntake";
        public static final String BACK_INTAKE_MOTOR = "backIntake";

        public static final DcMotorSimple.Direction FRONT_INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;
        public static final DcMotorSimple.Direction BACK_INTAKE_DIRECTION = DcMotorSimple.Direction.FORWARD;

        public static DcMotor.ZeroPowerBehavior INTAKE_ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.BRAKE;
        public static DcMotor.RunMode INTAKE_RUNMODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

        public static final double INTAKE_POWER = 1.0;
        public static final double EXTAKE_POWER = -1.0;
        public static final double IDLE_POWER = 0;
    }

    public static class LiftConstants{
        public static final String PTO_LEFT = "ptoLeft";
        public static final String PTO_RIGHT = "ptoRight";

        public static final double PTO_LOCK = 0.0;
        public static final double PTO_UNLOCK = 1;

        public static final Servo.Direction CW = Servo.Direction.FORWARD;
        public static final Servo.Direction CCW = Servo.Direction.REVERSE;

    }

    public static class TransferConstants {
        public static final String LIFT_SERVO_A = "flickA";
        public static final String LIFT_SERVO_B = "flickB";
        public static final String LIFT_SERVO_C = "flickC";


        public static final String INDEX_SENSOR_A = "indexSensorA";
        public static final String INDEX_SENSOR_B = "indexSensorB";
        public static final String INDEX_SENSOR_C = "indexSensorC";


        // Lift Constants - TODO TUNE THESE PLS
        public static final double FLICK_POS_UP = 160; // with POS 0 at the top, with cw rotation
        public static final double FLICK_POS_DOWN = 30; // with POS 0 at the top, with cw rotation

    }
    // Controller Constants
//    public static class ControllerConstants {
//        // Controller deadzones
//        public static final double LEFT_STICK_DEADZONE = 0.1;
//        public static final double RIGHT_STICK_DEADZONE = 0.1;
//        public static final double TRIGGER_DEADZONE = 0.1;
//
//        // Button mappings - Gamepad1 (Driver)
//        public static final int PRECISION_MODE_BUTTON = gamepad1.right_trigger;
//        public static final int TURBO_MODE_BUTTON = gamepad1.left_trigger;
//        public static final int FIELD_CENTRIC_TOGGLE_BUTTON = gamepad1.x;
//        public static final int RESET_IMU_BUTTON = gamepad1.y;
//
//        // Button mappings - Gamepad2 (Operator)
//        public static final int INTAKE_BUTTON = gamepad2.a;
//        public static final int OUTTAKE_BUTTON = gamepad2.b;
//        public static final int LIFT_UP_BUTTON = gamepad2.dpad_up;
//        public static final int LIFT_DOWN_BUTTON = gamepad2.dpad_down;
//    }
}