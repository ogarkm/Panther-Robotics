package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder; // Necessary for tab1, tab2, tab3
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.MecanumDrive; // <-- NEW IMPORT

@Config
@Autonomous(name = "BLUE_TEST_AUTO_PIXEL", group = "Autonomous")
public class SampleAuton extends LinearOpMode {

    // --- Component Classes (Lift and Claw) ---

    public class Lift {
        private final DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public class Claw {
        private final Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    // --- Autonomous Execution ---

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        // 1. Initialize Vision System
        AprilTagHelper aprilTagVision = new AprilTagHelper(hardwareMap);

        // Used to store the detected AprilTag ID (1, 2, or 3)
        int detectedTagId = 0;

        // Define Trajectories for each possible AprilTag ID (1, 2, 3)
        // These are defined as TrajectoryActionBuilders so we can select one later.
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);

        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);

        // Trajectory to close out the autonomous period (parking)
        Action trajectoryActionCloseOut = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(48, 12)) // Example parking location
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        Actions.runBlocking(claw.closeClaw());

        // 2. Vision Processing in the Init Loop
        while (!isStopRequested() && !opModeIsActive()) {
            // Continuously check for the AprilTag ID
            int currentDetection = aprilTagVision.getDetectedTagId();

            // If a tag (1, 2, or 3) is found, save it
            if (currentDetection != 0) {
                detectedTagId = currentDetection;
            }

            // Display the most recently confirmed detection
            telemetry.addData("Selected Tag ID (1, 2, or 3)", detectedTagId);
            telemetry.addData("Status", "Awaiting Start");
            telemetry.update();
        }

        // 3. Select Trajectory based on final detected position
        int startPosition = detectedTagId; // Use the final detected ID
        telemetry.addData("Starting Path for Tag", startPosition);
        telemetry.update();

        // Stop vision processing once we are out of the init loop
        aprilTagVision.close();

        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            // Default to position 3 if vision failed (detectedTagId=0) or detected 3
            trajectoryActionChosen = tab3.build();
        }

        // 4. Run the chosen sequence
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        lift.liftUp(),
                        claw.openClaw(),
                        lift.liftDown(),
                        trajectoryActionCloseOut
                )
        );
    }
}
