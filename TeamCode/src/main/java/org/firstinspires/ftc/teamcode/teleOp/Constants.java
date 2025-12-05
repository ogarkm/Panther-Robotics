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

        public static final DcMotorSimple.Direction FRONT_INTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
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
        public static final double FLICK_POS_UP = 1; // with POS 0 at the top, with cw rotation
        public static final double FLICK_POS_DOWN = 0; // with POS 0 at the top, with cw rotation



    }

    public static class TurretConstants {

        // Hood positions (TODO: TUNE THESE)
        public static final double HOOD_POS_CLOSE = 0.3;
        public static final double HOOD_POS_FAR = 0.7;
        public static final double HOOD_POS_MID = 0.5;


        // Rotation PID (for CR Servo tracking)
        public static final double ROTATION_KP = 0.025;
        public static final double ROTATION_KI = 0.0001;
        public static final double ROTATION_KD = 0.002;


        // Rotation limits
        public static final double MAX_ROTATION_POWER = 0.7;
        public static final double MIN_ROTATION_POWER = 0.12;
        public static final double ROTATION_DEADZONE = 2.0; // degrees

        public static final double SERVO_MIN = 0.0;
        public static final double SERVO_MAX = 1.0;

        public static final double MANUAL_TURRET_SPEED_DEG = 1.0;

        public static final double TURRET_POWER_MID = -0.6;
        public static final double TURRET_POWER_MAX = -1;
        public static final double TURRET_POWER_LOW = -0.37;

        public static final double SERVO_TO_TURRET_GEAR_RATIO = 3.5;
        public static final double TURRET_HOME_ANGLE = 0.0; // Home pos in deg (forward)

        public static final double TURN_GAIN = 0.02;

        public static final double EXTAKE_POWER = 0.3;


        public static final String LEFT_TURRET_SERVO = "turretservo";

        public static final String HOOD_TURRET_SERVO = "hoodservo";


        public static final String TURRET_RIGHT_MOTOR = "rturret";
        public static final String TURRET_LEFT_MOTOR = "lturret";

        public static final double kP = 0.02;
        public static final double kI = 0.00005;
        public static final double kD = 0.0015;
        public static final DcMotorSimple.Direction TURRET_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    }



}