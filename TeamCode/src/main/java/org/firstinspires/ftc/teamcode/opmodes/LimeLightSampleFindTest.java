package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.userjhansen.automap.PartType;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.galahlib.Button;
import org.firstinspires.ftc.teamcode.galahlib.actions.Loggable;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggableAction;
import org.firstinspires.ftc.teamcode.galahlib.actions.LoggingSequential;
import org.firstinspires.ftc.teamcode.galahlib.actions.Timeout;
import org.firstinspires.ftc.teamcode.localization.VisionDetection;
import org.firstinspires.ftc.teamcode.localization.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.localization.FuseLocation;
import org.firstinspires.ftc.teamcode.staticData.Logging;
import org.firstinspires.ftc.teamcode.staticData.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.StaticLights;

import java.util.List;
import java.util.concurrent.TimeUnit;

import com.acmerobotics.roadrunner.Pose2d;
import com.userjhansen.automap.AutoPart;

@TeleOp(name = "Vision: LimeLight", group = "Vision")
@Config
public class LimeLightSampleFindTest extends LinearOpMode {

    public static double TriggerMin = 0.01;

    Limelight3A limelight;

    public static TrajectoryActionBuilder addParts(TrajectoryActionBuilder traj, AutoPart[] parts) {
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value+0.00001);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }
        }
        return traj;
    }

    public static TrajectoryActionBuilder addActionAfterFirst(TrajectoryActionBuilder traj, AutoPart[] parts, Action action) {
        boolean addedFirst = false;
        for (AutoPart part : parts) {
            switch (part.type) {
                case STRAFE:
                    traj = traj.strafeTo(part.getPose().position);
                    break;
                case STRAFE_TO:
                    traj = traj.strafeToLinearHeading(part.getPose().position, part.getPose().heading);
                    break;
                case TURN:
                    traj = traj.turn(part.value);
                    break;
                case WAIT:
                    traj = traj.waitSeconds(part.value);
                    break;
                case SPLINE_TO:
                    traj = traj.splineToSplineHeading(part.getPose(), part.value);
                    break;
                case SPLINE_CONSTANT:
                    traj = traj.splineToConstantHeading(part.getPose().position, part.value);
                    break;
                case ACTION:
                    break;
                case CHANGE_LIGHT:
                    break;
            }

            if (!addedFirst) {
                addedFirst = true;
                traj = traj.afterTime(0.1, action);
            }
        }
        return traj;
    }

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!


        limelight.pipelineSwitch(2);

        MecanumDrive driveBase = new MecanumDrive(hardwareMap, PoseStorage.currentPose);
        VisionDetection visionDetection = new VisionDetection(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        DcMotorEx climbMotor = hardwareMap.get(DcMotorEx.class, "climb");
        StaticLights.setup(hardwareMap, "blinkin");

        LLResult result = limelight.getLatestResult();

        double tx = 0;
        double ty = 0;
        double ta = 0;

        if (result != null && result.isValid()) {
            tx = result.getTx(); // How far left or right the target is (degrees)
            ty = result.getTy(); // How far up or down the target is (degrees)
            ta = result.getTa(); // How big the target looks (0%-100% of the image)

            Logging.LOG("(limelight) Target X", tx);
            Logging.LOG("(limelight) Target Y", ty);
            Logging.LOG("(limelight) Target Area", ta);
            Logging.DEBUG("(limelight) Target X", tx);
            Logging.DEBUG("(limelight) Target Y", ty);
            Logging.DEBUG("(limelight) Target Area", ta);
        }


        double currentHeading = driveBase.localizer.currentPose.heading.toDouble();

        double distance;
        if (result.getTy() == 0) {
            // Handle edge case
            distance = Double.POSITIVE_INFINITY; // or some max range value
        } else {
            distance = (245 / Math.tan(Math.toRadians(result.getTy()) + 50)) / 25.4;
        }

        double targetHeading = currentHeading + Math.toRadians(result.getTx());


        Button fieldMode = new Button();
        Button slowMode = new Button();

        Button specimenReady = new Button();

        Button progressSample = new Button();
        Button progressSpecimen = new Button();

        Logging.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TelemetryPacket p;

        intake.lockout();
        outtake.lockout();

        PoseStorage.splitControls = true;
        while (!isStarted()) {
            p = new TelemetryPacket();
            driveBase.update(p);
            visionDetection.update(driveBase.localizer, p);
            Logging.update();
            StaticLights.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);

            if (gamepad2.back) {
                PoseStorage.splitControls = true;
            } else if (gamepad1.back) {
                PoseStorage.splitControls = false;
            }
        }

        if (isStopRequested()) return;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        LoggableAction sampleAction = null;
        LoggableAction finishingAction = new Loggable("INIT", new ParallelAction(
                outtake.homePosition(),
                intake.resetSlides(),
                outtake.resetLifts()
        ));
        boolean specimenGrabbed = false;
        SampleState sampleState = SampleState.Waiting;
        CapturingState captureState = CapturingState.None;
        FinishingState finishState = FinishingState.Outtake;

        intake.unlock();
        outtake.unlock();

        PoseStorage.isInit = false;
        Deadline matchTimer = new Deadline(2, TimeUnit.MINUTES);
        while (opModeIsActive() && !isStopRequested()) {
            p = new TelemetryPacket();

            Pose2d poseEstimate = driveBase.localizer.currentPose;

            if (gamepad1.dpad_right) {
                PoseStorage.isRedAlliance = true;
            } else if (gamepad1.dpad_left) {
                PoseStorage.isRedAlliance = false;
            }

            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).x) { // Blue only
                double[] inputs = {3, 0, 0, 0, 0, 0, 0, 0};
                limelight.updatePythonInputs(inputs);
                Logging.LOG("Colour Mode", "3");
                Logging.DEBUG("Colour Mode", "3");
            }

            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).b) { //Red only
                double[] inputs = {4, 0, 0, 0, 0, 0, 0, 0};
                limelight.updatePythonInputs(inputs);
                Logging.LOG("Colour Mode", "4");
                Logging.DEBUG("Colour Mode", "4");
            }


            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).y) {
                if (PoseStorage.isRedAlliance) { //Red & Yellow
                    double[] inputs = {2, 0, 0, 0, 0, 0, 0, 0};
                    limelight.updatePythonInputs(inputs);
                    Logging.LOG("Colour Mode", "2");
                    Logging.DEBUG("Colour Mode", "2");
                } else { //Blue & Yellow
                    double[] inputs = {1, 0, 0, 0, 0, 0, 0, 0};
                    limelight.updatePythonInputs(inputs);
                    Logging.LOG("Colour Mode", "1");
                    Logging.DEBUG("Colour Mode", "1");
                }
            }

            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).a) { //All Colours
                double[] inputs = {5, 0, 0, 0, 0, 0, 0, 0};
                limelight.updatePythonInputs(inputs);
                Logging.LOG("Colour Mode", "5");
                Logging.DEBUG("Colour Mode", "5");
            }


            float specificTrigger = (PoseStorage.splitControls ? gamepad2 : gamepad1).left_trigger;
            float sharedTrigger = (PoseStorage.splitControls ? gamepad2 : gamepad1).right_trigger;
            if ((specificTrigger > TriggerMin || sharedTrigger > TriggerMin)) {
                if (finishState == FinishingState.Intake) {
                    finishState = FinishingState.None;
                    finishingAction = null;
                }

                if (sampleAction == null) {
                    Logging.LOG("Running far capture sequence");
                    limelight.captureSnapshot("sample_pov");
                    result.getTx();
                    result.getTy();
                    boolean shared = sharedTrigger > TriggerMin;
                    TrajectoryActionBuilder builder = driveBase.allianceActionBuilder(new Pose2d(driveBase.localizer.currentPose.position.x, driveBase.localizer.currentPose.position.y, currentHeading));
                    builder = builder.turnTo(targetHeading - currentHeading);
                    sampleAction = intake.captureSample(shared, () -> distance);
                    captureState = CapturingState.Far;
                }
            } else if (captureState == CapturingState.Far) {
                sampleAction = null;
                captureState = CapturingState.None;
                finishState = FinishingState.Intake;
                finishingAction = intake.cancelSlides();
                limelight.deleteSnapshots();
                Logging.LOG("Snapshots", "All cleared out!");
                Logging.DEBUG("Snapshots", "All cleared out!");
            }


            Logging.LOG("SAMPLE_STATE", sampleState);
            if (sampleAction != null) {
                Logging.DEBUG("SAMPLE_ACTION", sampleAction.getName());

                if (!sampleAction.run(p)) {
                    Logging.LOG("SAMPLE_ACTION_FINISHED");
                    sampleAction = null;

                    switch (sampleState) {
                        case Waiting:
                            StaticLights.getColors()[0] = StaticLights.getColors()[1];
                            sampleState = SampleState.Captured;
                            captureState = CapturingState.None;
                            sampleAction = new LoggingSequential("TRANSFER",
                                    intake.retractSlides(),
                                    intake.transfer(),
                                    new Loggable("WAIT_FOR_IT", new SleepAction(1)),
                                    outtake.pickupInternalSample(),
                                    intake.stopTransfer(),
                                    outtake.pickupInternalSample()
                            );
                            break;

                        case SpecimenIntake:
                            sampleState = SampleState.Waiting;
                            finishState = FinishingState.Outtake;
                            finishingAction = outtake.returnSpecimen();
                            break;
                    }
                }
            }

            if ((PoseStorage.splitControls ? gamepad2 : gamepad1).start && sampleState == SampleState.Captured) {
                sampleState = SampleState.Waiting;
            }



            if (progressSpecimen.update((PoseStorage.splitControls ? gamepad2 : gamepad1).left_bumper)) {
                if (sampleState == SampleState.SpecimenWait) {
                    specimenGrabbed = !specimenGrabbed;
                    sampleAction = outtake.grabber(!specimenGrabbed);
                } else if (sampleState == SampleState.Captured) {
                    sampleAction = new LoggingSequential("TRANSFER",
                            new Loggable("FLIP_INTAKE_DOWN", intake.getFlipServo().setPosition(true)),
                            outtake.homePosition(),
                            intake.retractSlides(),
                            intake.transfer(),
                            new Loggable("WAIT_FOR_IT", new SleepAction(1)),
                            outtake.pickupInternalSample(),
                            intake.stopTransfer()
                    );
                } else if (sampleState == SampleState.SpecimenIntake) {
                    sampleAction = null;
                    finishingAction = outtake.raiseSpecimen();
                }
            }


            if (progressSample.update((PoseStorage.splitControls ? gamepad2 : gamepad1).right_bumper) && sampleAction == null) {
                switch (sampleState) {
                    case Captured:
                        sampleAction = outtake.topBasket();
                        sampleState = SampleState.Outtaking;
                        break;
                    case Outtaking:
                        sampleAction = outtake.dropSample();
                        sampleState = SampleState.Clearing;
                        break;
                    case Clearing:
                        finishingAction = outtake.retractArm();
                        finishState = FinishingState.Outtake;
                        sampleState = SampleState.Waiting;
                        break;
                    case SpecimenWait:
                        if (specimenGrabbed) {
                            finishingAction = outtake.raiseSpecimen(true);
                            sampleState = SampleState.SpecimenIntake;
                        } else {
                            finishingAction = outtake.abortSpecimen();
                            finishState = FinishingState.Outtake;
                            sampleState = SampleState.Waiting;
                        }
                        break;
                    case SpecimenIntake:
                        finishingAction = null;
                        sampleAction = outtake.ensureSpecimenPlaced();
                }
            }


            Logging.LOG("FINISH_STATE", finishState);
            if (finishingAction != null) {
                Logging.DEBUG("FINISH_ACTION", finishingAction.getName());

                if (!finishingAction.run(p)) {
                    Logging.LOG("FINISH_ACTION_FINISHED");
                    finishingAction = null;
                    finishState = FinishingState.None;
                }
            }

            Logging.LOG("CAPTURE_STATE", captureState);
            Logging.LOG("CURRENT_TEAM", PoseStorage.isRedAlliance ? "RED" : "BLUE");
            Logging.LOG("SPLIT", PoseStorage.splitControls);
            Logging.LOG("FIELD_MODE", fieldMode.val);
            Logging.LOG("SLOW_MODE", slowMode.val);
            Logging.LOG("HALLUCINATING", PoseStorage.shouldHallucinate);

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }
            driveBase.logState("[TELEOP]");
            intake.logState("[TELEOP]");
            outtake.logState("[TELEOP]");
            Logging.update();
            StaticLights.update();
            FtcDashboard.getInstance().sendTelemetryPacket(p);


//            TrajectoryActionBuilder builder = driveBase.allianceActionBuilder(new Pose2d(driveBase.localizer.currentPose.position.x, driveBase.localizer.currentPose.position.y, currentHeading));
//
//
//            builder = builder.turnTo(targetHeading - currentHeading)
//
//
//            Action capture = builder.build();



        }
    }

    enum SampleState {
        Waiting,
        Captured,
        Outtaking,
        Clearing,

        SpecimenWait,
        SpecimenIntake,
    }

    enum CapturingState {
        Close,
        Far,
        None
    }

    enum FinishingState {
        Intake,
        Outtake,
        None
    }
}
