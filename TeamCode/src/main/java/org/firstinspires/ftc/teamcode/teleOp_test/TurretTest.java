// #---------------------------#
//  Basic TeleOP with Turret Control (Should be Automatic tho)
//
//  FTC Team 12926 - Panther Robotics
//  Aarush Karnik
//
//  Adapted TeleOP from Game Manual 0
//  https://gm0.org
//  IDK if this actually works
// #---------------------------#

package org.firstinspires.ftc.teamcode.teleOp_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Turret Subsystem TeleOP", group="TeleOp")
public class TurretTest extends LinearOpMode {

    // Vision and AprilTag
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Turret Constants
    private static final double MANUAL_TURRET_SPEED_DEG = 1.0;
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_TO_TURRET_GEAR_RATIO = 3.5;
    private static final double TURRET_HOME_ANGLE = 0.0; // Home pos in deg (forward)

    final double TURN_GAIN = 0.02;

    // State machine modes
    private static final int STATE_SELECT_TEAM = 0;
    private static final int STATE_SELECT_MODE = 1;
    private static final int STATE_MANUAL_RUN = 2;
    private static final int STATE_AUTO_RUN = 3;

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        DcMotor frontLeftMotor = null;
        DcMotor backLeftMotor = null;
        DcMotor frontRightMotor = null;
        DcMotor backRightMotor = null;
        Servo turretLeft = null;
        Servo turretRight = null;

        try {
            frontLeftMotor = hardwareMap.dcMotor.get("fl");
            backLeftMotor = hardwareMap.dcMotor.get("bl");
            frontRightMotor = hardwareMap.dcMotor.get("fr");
            backRightMotor = hardwareMap.dcMotor.get("br");
            turretLeft = hardwareMap.get(Servo.class, "turretLeft");

            // Attempt to get the second servo, but don't fail if it's not there.
            try {
                turretRight = hardwareMap.get(Servo.class, "turretRight");
            } catch (Exception e) {
                telemetry.addLine("Single servo mode (turretRight not found)");
            }

        } catch (Exception e) {
            telemetry.addLine("CRITICAL ERROR: A required hardware device is missing.");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            // Terminate the OpMode gracefully if essential hardware is missing.
            sleep(5000);
            return;
        }

        // --- AprilTag Initialization ---
        initAprilTag();

        // --- Motor Configuration ---
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


        // --- Variable Initialization ---
        double turretAngle = TURRET_HOME_ANGLE;
        int currentState = STATE_SELECT_TEAM;
        int targetTagId = -1;

        telemetry.addLine("Initialization Complete. Awaiting Start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Robot Driving Logic (runs in all states)
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            // --- State Machine :) ---
            // TODO: Needs to be hardcoded to TELEOP for COMP

            // State 0: Choose Alliance
            if (currentState == STATE_SELECT_TEAM) {
                telemetry.addLine("Press X for Blue team, Y for Red team");
                if (gamepad1.x) {
                    targetTagId = 20; // Example Blue Alliance Tag ID
                    currentState = STATE_SELECT_MODE;
                } else if (gamepad1.y) {
                    targetTagId = 24; // Example Red Alliance Tag ID
                    currentState = STATE_SELECT_MODE;
                }
            }
            // State 1: Choose Turret Mode
            else if (currentState == STATE_SELECT_MODE) {
                telemetry.addLine("Press A for Manual Turret, B for Auto Turret");
                if (gamepad1.a) {
                    currentState = STATE_MANUAL_RUN;
                } else if (gamepad1.b) {
                    currentState = STATE_AUTO_RUN;
                }
            }
            // State 2: Manual Turret Control
            else if (currentState == STATE_MANUAL_RUN) {
                telemetry.addData("Mode", "Manual");
                double turretInput = 0.0;
                if (gamepad1.left_bumper) {
                    turretInput = -1.0;
                } else if (gamepad1.right_bumper) {
                    turretInput = 1.0;
                }
                turretAngle += turretInput * MANUAL_TURRET_SPEED_DEG;
            }

            // State 3: Automatic Turret Control
            else if (currentState == STATE_AUTO_RUN) {
                telemetry.addData("Mode", "Auto");
                AprilTagDetection targetTag = null;
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null && detection.id == targetTagId) {
                        targetTag = detection;
                        break; // Found the target, stop searching
                    }
                }

                if (targetTag != null) {
                    double yawError = targetTag.ftcPose.yaw;
                    turretAngle -= (yawError * TURN_GAIN);

                    telemetry.addData("Target Found", "ID: %d", targetTag.id);
                    telemetry.addData("Yaw Error", "%.2f degrees", yawError);
                } else {
                    turretAngle = TURRET_HOME_ANGLE;
                    telemetry.addLine("Target not visible. Returning to home.");
                }
            }

            if ((currentState == STATE_MANUAL_RUN || currentState == STATE_AUTO_RUN) && gamepad1.start) {
                turretAngle = TURRET_HOME_ANGLE;
            }


            double wrappedAngle = (turretAngle % 360 + 360) % 360;

            double servoPos = getServoPos(wrappedAngle);

            turretLeft.setPosition(servoPos);
            if (turretRight != null) {
                turretRight.setPosition(servoPos);
            }


            // --- Telemetry ---
            telemetry.addData("Turret Angle (Logical)", "%.2f", turretAngle);
            telemetry.addData("Turret Angle (Wrapped)", "%.2f", wrappedAngle);
            telemetry.addData("Servo Position", "%.3f", servoPos);
            telemetry.update();
        }
    }

    private static double getServoPos(double wrappedAngle) {
        // --- Servo Position Calculation and Command ---
        double normalizedTurretPos = wrappedAngle / 360.0;
        double servoRotations = normalizedTurretPos * SERVO_TO_TURRET_GEAR_RATIO;

        double servoPos = servoRotations % 1.0;

        servoPos = Range.clip(servoPos, SERVO_MIN, SERVO_MAX);
        return servoPos;
    }

    private void initAprilTag() {
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