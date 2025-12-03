// #---------------------------#
//  Basic Mecanum TeleOP with Odometry
//
//  Adapted from Game Manual 0
//  https://gm0.org
//
// #---------------------------#

package org.firstinspires.ftc.teamcode.teleOp_test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp(name="ODOMETRY_TEST", group="TEST")
public class OdometryTest extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 537.7; // TODO: TUNE THIS
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;  // TODO: TUNE THIS
    static final double TRACK_WIDTH_INCHES = 15.0; // TODO: TUNE THIS
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                          (WHEEL_DIAMETER_INCHES * Math.PI);

    private double robotX = 12.0;
    private double robotY = 12.0;
    private double robotHeading = Math.PI / 2;

    private int lastFrontLeftPos;
    private int lastBackLeftPos;
    private int lastFrontRightPos;
    private int lastBackRightPos;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("lf");
        backLeftMotor = hardwareMap.dcMotor.get("lb");
        frontRightMotor = hardwareMap.dcMotor.get("rf");
        backRightMotor = hardwareMap.dcMotor.get("rb");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
        waitForStart();
        if (isStopRequested()) return;

        lastFrontLeftPos = frontLeftMotor.getCurrentPosition();
        lastBackLeftPos = backLeftMotor.getCurrentPosition();
        lastFrontRightPos = frontRightMotor.getCurrentPosition();
        lastBackRightPos = backRightMotor.getCurrentPosition();

        while (opModeIsActive()) {
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

            updateOdometry();
            updateTelemetry();
        }
    }

    public void resetEncoders() {
        // Reset Encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set to RUN_WITHOUT_ENCODER
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void updateOdometry() {
        // Get current positions
        int currentFrontLeftPos = frontLeftMotor.getCurrentPosition();
        int currentBackLeftPos = backLeftMotor.getCurrentPosition();
        int currentFrontRightPos = frontRightMotor.getCurrentPosition();
        int currentBackRightPos = backRightMotor.getCurrentPosition();
        // Calculate deltas
        int deltaFrontLeft = currentFrontLeftPos - lastFrontLeftPos;
        int deltaBackLeft = currentBackLeftPos - lastBackLeftPos;
        int deltaFrontRight = currentFrontRightPos - lastFrontRightPos;
        int deltaBackRight = currentBackRightPos - lastBackRightPos;
        // Convert to inches
        double deltaFrontLeftInches = deltaFrontLeft / COUNTS_PER_INCH;
        double deltaBackLeftInches = deltaBackLeft / COUNTS_PER_INCH;
        double deltaFrontRightInches = deltaFrontRight / COUNTS_PER_INCH;
        double deltaBackRightInches = deltaBackRight / COUNTS_PER_INCH;

        double deltaY_robot = (deltaFrontLeftInches + deltaBackLeftInches + deltaFrontRightInches + deltaBackRightInches) / 4.0;
        double deltaX_robot = (-deltaFrontLeftInches + deltaBackLeftInches + deltaFrontRightInches - deltaBackRightInches) / 4.0;
        double deltaHeading = (deltaFrontRightInches + deltaBackRightInches - deltaFrontLeftInches - deltaBackLeftInches) / (2.0 * TRACK_WIDTH_INCHES);

        double deltaX_field = deltaX_robot * Math.cos(robotHeading) - deltaY_robot * Math.sin(robotHeading);
        double deltaY_field = deltaX_robot * Math.sin(robotHeading) + deltaY_robot * Math.cos(robotHeading);

        robotX += deltaX_field;
        robotY += deltaY_field;
        robotHeading += deltaHeading;

        lastFrontLeftPos = currentFrontLeftPos;
        lastBackLeftPos = currentBackLeftPos;
        lastFrontRightPos = currentFrontRightPos;
        lastBackRightPos = currentBackRightPos;
    }

    private boolean isInsideLaunchZone(double x, double y) {
        double x1 = 0, y1 = 144;
        double x2 = 144, y2 = 144;
        double x3 = 72, y3 = 72;

        double d1 = (x - x2) * (y3 - y2) - (x3 - x2) * (y - y2);
        double d2 = (x - x3) * (y1 - y3) - (x1 - x3) * (y - y3);
        double d3 = (x - x1) * (y2 - y1) - (x2 - x1) * (y - y1);

        boolean has_neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        boolean has_pos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(has_neg && has_pos);
    }

    public void updateTelemetry() {
        int tileX = (int) Math.floor(robotX / 24.0) + 1;
        int tileY = (int) Math.floor(robotY / 24.0) + 1;
        tileX = Math.max(1, Math.min(6, tileX));
        tileY = Math.max(1, Math.min(6, tileY));

        boolean inLaunchZone = isInsideLaunchZone(robotX, robotY);

        telemetry.addData("Position", String.format(Locale.US, "(%.2f, %.2f)", robotX, robotY));
        telemetry.addData("In Launch Zone", inLaunchZone);
        telemetry.addData("Tile Position", String.format("(%d, %d)", tileX, tileY));
        telemetry.update();
    }
}