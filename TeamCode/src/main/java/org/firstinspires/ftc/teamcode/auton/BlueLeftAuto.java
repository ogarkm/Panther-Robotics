package org.firstinspires.ftc.teamcode.auton;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions; // may or may not be present; see note below

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hware.hwMap;
import org.firstinspires.ftc.teamcode.subsystems.TransferSys;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "BlueLeftAuto", group = "Autonomous")
public class BlueLeftAuto extends LinearOpMode {

    // Hardware and subsystems
    private MecanumDrive drive;
    private hwMap.TransferHwMap transferHw;
    private hwMap.TurretHwMap turretHw;

    private TransferSys transfer;
    private Turret turret;

    private AprilTagHelper tagHelper;
    private int detectedTagId = 0;

    // Trajectory/Action placeholders
    private TrajectoryActionBuilder tab1;
    private TrajectoryActionBuilder tab2;
    private TrajectoryActionBuilder tab3;

    // Action objects that will be run at start()
    private Action trajectoryAction1;
    private Action trajectoryAction2;
    private Action trajectoryAction3;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1) Instantiate hardware maps from your hwMap class
        transferHw = new hwMap.TransferHwMap(hardwareMap);
        turretHw = new hwMap.TurretHwMap(hardwareMap);

        // 2) Instantiate subsystem wrappers
        transfer = new TransferSys(transferHw);
        turret = new Turret(turretHw);

        // 3) Instantiate RoadRunner MecanumDrive with a reasonable initial pose.
        //    The guide uses an initial pose based on field coordinates; adjust these to match your robot's start.
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90)); // example from guide
        drive = new MecanumDrive(hardwareMap, initialPose);

        // 4) Prepare AprilTag helper for init-loop detection
        tagHelper = new AprilTagHelper(hardwareMap);

        telemetry.addLine("Initialized hardware and subsystems.");
        telemetry.update();

        //
        // === BUILD TRAJECTORIES DURING INIT (expensive — do it before start) ===
        //

        // TrajectoryActionBuilder 1 (example: path that goes to scoring area A, then cycles)
        tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(0.2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(0.6);

        // TrajectoryActionBuilder 2 (a different path mapped to tag 2)
        tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(0.4)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(0.4);

        // TrajectoryActionBuilder 3 (a third path mapped to tag 3)
        tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(0.2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(0.4);

        // Final trajectory variations (closeout / park) — you can use endTrajectory() chaining if desired
        // Build Action objects now so we can run them quickly at start()
        trajectoryAction1 = tab1.build(); // <--- produces an Action (RoadRunner 1.0)
        trajectoryAction2 = tab2.build();
        trajectoryAction3 = tab3.build();

        telemetry.addLine("Trajectories built. Waiting for tag detection...");
        telemetry.update();

        // INIT LOOP: run vision until start - read tag in this loop
        while (!isStopRequested() && !isStarted()) {
            int tag = tagHelper.getDetectedTagId();
            if (tag != 0) {
                detectedTagId = tag;
            }
            telemetry.addData("Tag seen (live)", tag);
            telemetry.addData("Tag picked (will use at start)", detectedTagId);
            telemetry.update();
        }

        // After the user presses start, free vision resources (optional but recommended)
        tagHelper.close();

        // ===== START =====
        waitForStart(); // if using LinearOpMode, ensure this is in correct place; in many templates you call waitForStart() earlier.
        if (isStopRequested()) return;

        // Example: pre-score/prep actions before moving (block/claw/lift) using your Transfer/Turret
        // These are simple synchronous calls - adapt to your mechanisms' Action wrappers if you have them.
        transfer.setTransferState(TransferSys.TransferState.INDEXING); // prepare indexing
        turret.setTurretState(Turret.TurretState.LAUNCH); // spin up shooter if needed

        // Small delay to let mechanisms spin up (if necessary)
        sleep(300);

        // Run the trajectory chosen by the last-detected tag (safe fallback to 2)
        int tagToUse = (detectedTagId == 0) ? 2 : detectedTagId;

        telemetry.addData("Running auto for tag", tagToUse);
        telemetry.update();

        // Now execute the selected action. The exact API to execute an Action object varies by codebase.
        // In many RoadRunner-based projects you will use:
        //    Actions.runBlocking(trajectoryActionX);
        //
        // Or your MecanumDrive may expose a helper like:
        //    drive.runAction(trajectoryActionX);
        //
        // If neither exists in your MecanumDrive, adapt to the API present in your MecanumDrive (see methods).
        //
        // I provide both patterns below — uncomment the one that matches your codebase:

        if (tagToUse == 1) {
            // Option A: global Actions helper (com.acmerobotics.roadrunner.ftc.Actions)
            try {
                Actions.runBlocking(trajectoryAction1);
            } catch (NoClassDefFoundError | Exception e) {
                // Option B: fallback — call into drive-specific executor if you have one.
                // Replace `executeActionOnDrive` with the correct method your drive provides.
                // Example: drive.runAction(trajectoryAction1);
                // If your drive expects a different type, convert as required by your API.
                //noinspection StatementWithEmptyBody
                { /* replace with drive.runAction(trajectoryAction1); */ }
            }
        } else if (tagToUse == 2) {
            try {
                Actions.runBlocking(trajectoryAction2);
            } catch (NoClassDefFoundError | Exception e) {
                // replace with drive.runAction(trajectoryAction2);
            }
        } else { // tag 3
            try {
                Actions.runBlocking(trajectoryAction3);
            } catch (NoClassDefFoundError | Exception e) {
                // replace with drive.runAction(trajectoryAction3);
            }
        }

        // After drive finishes, do scoring routine
        // Example: flick one element then stop turret/transfer
        transfer.setTransferState(TransferSys.TransferState.FLICKING);
        sleep(500); // allow flick
        transfer.setTransferState(TransferSys.TransferState.IDLING);

        turret.setTurretState(Turret.TurretState.IDLE);

        telemetry.addLine("Auto complete.");
        telemetry.update();
    }
}
