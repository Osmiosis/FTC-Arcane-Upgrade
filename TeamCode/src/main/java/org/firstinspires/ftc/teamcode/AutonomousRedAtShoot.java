package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous
public class AutonomousRedAtShoot extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    // Shooter hardware
    private ShooterPID shooter;
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private DcMotor shooterIntake;
    private Servo blockServo;

    // Shooting state tracking
    private Timer shootTimer;
    private boolean shootingComplete = false;
    private Timer shootTimer2;
    private boolean shootingComplete2 = false;
    private Timer shootTimer3;
    private boolean shootingComplete3 = false;
    private Timer shootTimer4;
    private boolean shootingComplete4 = false;
    private static final double WAIT_AT_SHOOT_POSE = 2.5; // increased for consistency (3 pulses over 2.5s)

    // ── Intake pulse settings ──────────────────────────────────────────────────
    // DRIVING intake: pulses at 0.5s on / 0.5s off while collecting
    private static final double PULSE_ON_SECONDS  = 0.5;
    private static final double PULSE_OFF_SECONDS = 0.5;

    // SHOOTING intake: exactly 3 pulses over WAIT_AT_SHOOT_POSE seconds
    // Each pulse cycle = 2.5 / 3 = ~0.833s. On = Off = ~0.417s.
    private static final double SHOOT_PULSE_CYCLE = WAIT_AT_SHOOT_POSE / 3.0;
    private static final double SHOOT_PULSE_ON    = SHOOT_PULSE_CYCLE / 2.0;
    private static final double SHOOT_PULSE_OFF   = SHOOT_PULSE_CYCLE / 2.0;

    private Timer pulseTimer = new Timer();

    // ── Shooter speed ──────────────────────────────────────────────────────────
    private static final double SHOOTER_SPEED_SCALE = 0.83;

    /** Pulsed power while DRIVING to collect (0.5s on / 0.5s off). */
    private double getPulsedIntakePower() {
        double elapsed = pulseTimer.getElapsedTimeSeconds() % (PULSE_ON_SECONDS + PULSE_OFF_SECONDS);
        return elapsed < PULSE_ON_SECONDS ? Constants.SHOOTER_INTAKE_POWER : 0.0;
    }

    /** Pulsed power while SHOOTING — exactly 3 pulses over WAIT_AT_SHOOT_POSE seconds. */
    private double getShootingPulsePower() {
        double elapsed = pulseTimer.getElapsedTimeSeconds() % SHOOT_PULSE_CYCLE;
        return elapsed < SHOOT_PULSE_ON ? Constants.SHOOTER_INTAKE_POWER : 0.0;
    }

    /** Call this once when you want to START a new pulse cycle (resets the timer). */
    private void startIntakePulse() {
        pulseTimer.resetTimer();
    }

    /** Scaled shooter target to fix overshoot. */
    private double scaledShooterTarget() {
        return Constants.SHOOTER_TARGET * SHOOTER_SPEED_SCALE;
    }

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE

        DRIVE_STARTPOS_SHOOT_POS,
        ALIGN_TO_CITADEL,         // nudge to precise citadel-facing pose before preload shoot
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOSE_FIRSTCYCLEINTAKEPREP,
        INTAKE_PREP,
        DRIVE_INTAKEPREP_INTAKECOLLECT,
        INTAKE_COLLECT,
        DRIVE_INTAKECOLLECT_SHOOTPOSE,
        ALIGN_TO_CITADEL_CYCLE1,  // realign before cycle 1 shoot
        SHOOT_CYCLE1,
        DRIVE_SHOOTPOSE_SECONDCYCLEINTAKEPREP,
        SECOND_INTAKE_PREP,
        DRIVE_SECONDINTAKEPREP_SECONDINTAKECOLLECT,
        SECOND_INTAKE_COLLECT,
        DRIVE_SECONDINTAKECOLLECT_SHOOTPOSE,
        ALIGN_TO_CITADEL_CYCLE2,  // realign before cycle 2 shoot
        SHOOT_CYCLE2,
        DRIVE_SHOOTPOSE_THIRDCYCLEINTAKEPREP,
        THIRD_INTAKE_PREP,
        DRIVE_THIRDINTAKEPREP_THIRDINTAKECOLLECT,
        THIRD_INTAKE_COLLECT,
        DRIVE_THIRDINTAKECOLLECT_SHOOTPOSE,
        ALIGN_TO_CITADEL_CYCLE3,  // realign before cycle 3 shoot
        SHOOT_CYCLE3,
        DRIVE_TO_END_POSE,
        END
    }

    PathState pathState;

    private final Pose startPose = new Pose(123.235, 122.161, Math.toRadians(222));

    private PathChain driveStartPosShootPos;
    private PathChain alignToCitadel;            // precise citadel-facing alignment nudge
    private PathChain driveShootPosFirstCycleIntakePrep;
    private PathChain driveIntakePrepIntakeCollect;
    private PathChain driveIntakeCollectShootPos;
    // Cycle 2 paths
    private PathChain driveShootPosSecondCycleIntakePrep;
    private PathChain driveSecondIntakePrepSecondIntakeCollect;
    private PathChain driveSecondIntakeCollectShootPos;
    // Cycle 3 paths
    private PathChain driveShootPosThirdCycleIntakePrep;
    private PathChain driveThirdIntakePrepThirdIntakeCollect;
    private PathChain driveThirdIntakeCollectShootPos;
    // End path
    private PathChain driveShootPosEndPose;

    public void buildPaths() {
        // start -> shoot approach (constant 222°)
        // Bot moves fast and settles loosely — shooter is already spinning up
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(123.235, 122.161), new Pose(95.388, 91.712)))
                .setConstantHeadingInterpolation(Math.toRadians(222))
                .build();

        // citadel alignment nudge — short move to lock onto exact shoot pose facing the goal
        // 222° = roughly facing the citadel on the red side. Adjust X/Y if still offset.
        alignToCitadel = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.388, 91.712), new Pose(94.5, 93.0)))
                .setConstantHeadingInterpolation(Math.toRadians(222))
                .build();

        // shoot -> first intake prep (turn 222° -> 0°)
        // Nudged Y from 83.512 -> 80.5, shifted X +3 to match other intake lines
        driveShootPosFirstCycleIntakePrep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.388, 91.712), new Pose(95.388, 80.5)))
                .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                .build();

        // first intake prep -> first intake collect (constant 0°)
        // Shifted both endpoints +3 inches on X to match 2nd and 3rd intake lines
        driveIntakePrepIntakeCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.388, 80.5), new Pose(132.0, 80.5)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // first intake collect -> shoot (turn 0° -> 222°)
        driveIntakeCollectShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.0, 80.5), new Pose(95.388, 91.712)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
                .build();

        // shoot -> second intake prep (turn 222° -> 0°)
        // +3 inches on intake prep X to shift line deeper toward balls
        driveShootPosSecondCycleIntakePrep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.388, 91.712), new Pose(95.319, 60.208)))
                .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                .build();

        // second intake prep -> second intake collect (constant 0°)
        // +3 inches on both ends
        driveSecondIntakePrepSecondIntakeCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.319, 60.208), new Pose(128.726, 59.861)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // second intake collect -> shoot (turn 0° -> 222°)
        driveSecondIntakeCollectShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128.726, 59.861), new Pose(95.388, 91.712)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
                .build();

        // shoot -> third intake prep (turn 222° -> 0°)
        // +3 inches on intake prep X to shift line deeper toward balls
        driveShootPosThirdCycleIntakePrep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.388, 91.712), new Pose(95.368, 35.640)))
                .setLinearHeadingInterpolation(Math.toRadians(222), Math.toRadians(0))
                .build();

        // third intake prep -> third intake collect (constant 0°)
        // +3 inches on both ends
        driveThirdIntakePrepThirdIntakeCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.368, 35.640), new Pose(128.033, 35.557)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // third intake collect -> shoot (turn 0° -> 222°)
        driveThirdIntakeCollectShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(128.033, 35.557), new Pose(95.388, 91.712)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(222))
                .build();

        // shoot -> end pose (constant 222°)
        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(95.388, 91.712), new Pose(84.939, 101.499)))
                .setConstantHeadingInterpolation(Math.toRadians(222))
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                // Shooter spins up while driving — arrives ready to shoot
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Shooter Status", "Spinning up during travel");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                telemetry.addData("Shooter Velocity", "M1:%.0f M2:%.0f",
                        shooter.getMotorVelocities()[0], shooter.getMotorVelocities()[1]);
                if (!follower.isBusy()) {
                    follower.followPath(alignToCitadel, true);
                    setTransitionPathState(PathState.ALIGN_TO_CITADEL);
                }
                break;

            case ALIGN_TO_CITADEL:
                // Short nudge to lock onto exact citadel-facing pose
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Aligning to citadel...");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                shooter.update();
                double err0 = Math.abs(shooter.getError());

                if (err0 <= 200.0 && !shootingComplete) {
                    // ── SHOOTING PULSE: 3 pulses over 2s ──
                    shooterIntake.setPower(getShootingPulsePower());
                    if (shootTimer == null) {
                        shootTimer = new Timer();
                        shootTimer.resetTimer();
                        startIntakePulse(); // begin pulse cycle
                    }
                    if (shootTimer.getElapsedTimeSeconds() >= WAIT_AT_SHOOT_POSE) {
                        shooterIntake.setPower(0);
                        shooter.setTarget(0);
                        shooter.update();
                        shootingComplete = true;
                        follower.followPath(driveShootPosFirstCycleIntakePrep, true);
                        setTransitionPathState(PathState.DRIVE_SHOOTPOSE_FIRSTCYCLEINTAKEPREP);
                    } else {
                        telemetry.addData("Shooting Timer", "%.1f / %.1f", shootTimer.getElapsedTimeSeconds(), WAIT_AT_SHOOT_POSE);
                    }
                } else if (!shootingComplete) {
                    shooterIntake.setPower(0);
                    telemetry.addData("Shooter Status", "Waiting for speed");
                }
                telemetry.addData("Shooter Error", "%.0f", err0);
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "PULSING" : "STOPPED");
                break;

            case DRIVE_SHOOTPOSE_FIRSTCYCLEINTAKEPREP:
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to first intake prep");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.INTAKE_PREP);
                }
                break;

            case INTAKE_PREP:
                startIntakePulse(); // begin fresh pulse cycle
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "First intake prep - intake pulsing");
                follower.followPath(driveIntakePrepIntakeCollect, true);
                setTransitionPathState(PathState.DRIVE_INTAKEPREP_INTAKECOLLECT);
                break;

            case DRIVE_INTAKEPREP_INTAKECOLLECT:
                // ── PULSED intake while driving to collect ──
                shooterIntake.setPower(getPulsedIntakePower());
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Driving to first intake collect");
                telemetry.addData("Intake", shooterIntake.getPower() > 0 ? "PULSING ON" : "PULSING OFF");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.INTAKE_COLLECT);
                }
                break;

            case INTAKE_COLLECT:
                shooterIntake.setPower(0);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "First collect done - returning to shooter");
                follower.followPath(driveIntakeCollectShootPos, true);
                setTransitionPathState(PathState.DRIVE_INTAKECOLLECT_SHOOTPOSE);
                break;

            case DRIVE_INTAKECOLLECT_SHOOTPOSE:
                // Shooter spins up while driving back — arrives ready to shoot
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Returning to shooter - spinning up");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    follower.followPath(alignToCitadel, true);
                    setTransitionPathState(PathState.ALIGN_TO_CITADEL_CYCLE1);
                }
                break;

            case ALIGN_TO_CITADEL_CYCLE1:
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Aligning to citadel (cycle 1)...");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_CYCLE1);
                }
                break;

            case SHOOT_CYCLE1:
                shooter.update();
                double err1 = Math.abs(shooter.getError());

                if (err1 <= 200.0 && !shootingComplete2) {
                    // ── SHOOTING PULSE: 3 pulses over 2s ──
                    shooterIntake.setPower(getShootingPulsePower());
                    if (shootTimer2 == null) {
                        shootTimer2 = new Timer();
                        shootTimer2.resetTimer();
                        startIntakePulse();
                    }
                    if (shootTimer2.getElapsedTimeSeconds() >= WAIT_AT_SHOOT_POSE) {
                        shooterIntake.setPower(0);
                        shooter.setTarget(0);
                        shooter.update();
                        shootingComplete2 = true;
                        follower.followPath(driveShootPosSecondCycleIntakePrep, true);
                        setTransitionPathState(PathState.DRIVE_SHOOTPOSE_SECONDCYCLEINTAKEPREP);
                    } else {
                        telemetry.addData("Cycle1 Timer", "%.1f / %.1f", shootTimer2.getElapsedTimeSeconds(), WAIT_AT_SHOOT_POSE);
                    }
                } else if (!shootingComplete2) {
                    shooterIntake.setPower(0);
                    telemetry.addData("Shooter Status", "Waiting for speed");
                }
                telemetry.addData("Shooter Error", "%.0f", err1);
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "PULSING" : "STOPPED");
                break;

            case DRIVE_SHOOTPOSE_SECONDCYCLEINTAKEPREP:
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to second intake prep");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SECOND_INTAKE_PREP);
                }
                break;

            case SECOND_INTAKE_PREP:
                startIntakePulse();
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Second intake prep - intake pulsing");
                follower.followPath(driveSecondIntakePrepSecondIntakeCollect, true);
                setTransitionPathState(PathState.DRIVE_SECONDINTAKEPREP_SECONDINTAKECOLLECT);
                break;

            case DRIVE_SECONDINTAKEPREP_SECONDINTAKECOLLECT:
                // ── PULSED intake while driving to collect ──
                shooterIntake.setPower(getPulsedIntakePower());
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Driving to second intake collect");
                telemetry.addData("Intake", shooterIntake.getPower() > 0 ? "PULSING ON" : "PULSING OFF");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SECOND_INTAKE_COLLECT);
                }
                break;

            case SECOND_INTAKE_COLLECT:
                shooterIntake.setPower(0);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Second collect done - returning to shooter");
                follower.followPath(driveSecondIntakeCollectShootPos, true);
                setTransitionPathState(PathState.DRIVE_SECONDINTAKECOLLECT_SHOOTPOSE);
                break;

            case DRIVE_SECONDINTAKECOLLECT_SHOOTPOSE:
                // Shooter spins up while driving back — arrives ready to shoot
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Returning to shooter (cycle2) - spinning up");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    follower.followPath(alignToCitadel, true);
                    setTransitionPathState(PathState.ALIGN_TO_CITADEL_CYCLE2);
                }
                break;

            case ALIGN_TO_CITADEL_CYCLE2:
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Aligning to citadel (cycle 2)...");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_CYCLE2);
                }
                break;

            case SHOOT_CYCLE2:
                shooter.update();
                double err2 = Math.abs(shooter.getError());

                if (err2 <= 200.0 && !shootingComplete3) {
                    // ── SHOOTING PULSE: 3 pulses over 2s ──
                    shooterIntake.setPower(getShootingPulsePower());
                    if (shootTimer3 == null) {
                        shootTimer3 = new Timer();
                        shootTimer3.resetTimer();
                        startIntakePulse();
                    }
                    if (shootTimer3.getElapsedTimeSeconds() >= WAIT_AT_SHOOT_POSE) {
                        shooterIntake.setPower(0);
                        shooter.setTarget(0);
                        shooter.update();
                        shootingComplete3 = true;
                        follower.followPath(driveShootPosThirdCycleIntakePrep, true);
                        setTransitionPathState(PathState.DRIVE_SHOOTPOSE_THIRDCYCLEINTAKEPREP);
                    } else {
                        telemetry.addData("Cycle2 Timer", "%.1f / %.1f", shootTimer3.getElapsedTimeSeconds(), WAIT_AT_SHOOT_POSE);
                    }
                } else if (!shootingComplete3) {
                    shooterIntake.setPower(0);
                    telemetry.addData("Shooter Status", "Waiting for speed");
                }
                telemetry.addData("Shooter Error", "%.0f", err2);
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "PULSING" : "STOPPED");
                break;

            case DRIVE_SHOOTPOSE_THIRDCYCLEINTAKEPREP:
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to third intake prep");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.THIRD_INTAKE_PREP);
                }
                break;

            case THIRD_INTAKE_PREP:
                startIntakePulse();
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Third intake prep - intake pulsing");
                follower.followPath(driveThirdIntakePrepThirdIntakeCollect, true);
                setTransitionPathState(PathState.DRIVE_THIRDINTAKEPREP_THIRDINTAKECOLLECT);
                break;

            case DRIVE_THIRDINTAKEPREP_THIRDINTAKECOLLECT:
                // ── PULSED intake while driving to collect ──
                shooterIntake.setPower(getPulsedIntakePower());
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Driving to third intake collect");
                telemetry.addData("Intake", shooterIntake.getPower() > 0 ? "PULSING ON" : "PULSING OFF");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.THIRD_INTAKE_COLLECT);
                }
                break;

            case THIRD_INTAKE_COLLECT:
                shooterIntake.setPower(0);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Third collect done - returning to shooter");
                follower.followPath(driveThirdIntakeCollectShootPos, true);
                setTransitionPathState(PathState.DRIVE_THIRDINTAKECOLLECT_SHOOTPOSE);
                break;

            case DRIVE_THIRDINTAKECOLLECT_SHOOTPOSE:
                // Shooter spins up while driving back — arrives ready to shoot
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Returning to shooter (cycle3) - spinning up");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    follower.followPath(alignToCitadel, true);
                    setTransitionPathState(PathState.ALIGN_TO_CITADEL_CYCLE3);
                }
                break;

            case ALIGN_TO_CITADEL_CYCLE3:
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Aligning to citadel (cycle 3)...");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_CYCLE3);
                }
                break;

            case SHOOT_CYCLE3:
                shooter.update();
                double err3 = Math.abs(shooter.getError());
                if (err3 <= 200.0 && !shootingComplete4) {
                    // ── SHOOTING PULSE: 3 pulses over 2s ──
                    shooterIntake.setPower(getShootingPulsePower());
                    if (shootTimer4 == null) {
                        shootTimer4 = new Timer();
                        shootTimer4.resetTimer();
                        startIntakePulse();
                    }
                    if (shootTimer4.getElapsedTimeSeconds() >= WAIT_AT_SHOOT_POSE) {
                        shooterIntake.setPower(0);
                        shooter.setTarget(0);
                        shooter.update();
                        shootingComplete4 = true;
                        follower.followPath(driveShootPosEndPose, true);
                        setTransitionPathState(PathState.DRIVE_TO_END_POSE);
                    } else {
                        telemetry.addData("Cycle3 Timer", "%.1f / %.1f", shootTimer4.getElapsedTimeSeconds(), WAIT_AT_SHOOT_POSE);
                    }
                } else if (!shootingComplete4) {
                    shooterIntake.setPower(0);
                    telemetry.addData("Shooter Status", "Waiting for speed");
                }
                telemetry.addData("Shooter Error", "%.0f", err3);
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "PULSING" : "STOPPED");
                break;

            case DRIVE_TO_END_POSE:
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to end pose");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.END);
                }
                break;

            case END:
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Autonomous Complete!");
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }
    }


    public void setTransitionPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);

        // Initialize Shooter motors
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize Shooter PID controller
        shooter = new ShooterPID(shooterMotor1, shooterMotor2);

        // Initialize Shooter Intake
        shooterIntake = hardwareMap.get(DcMotor.class, "shooter_intake");
        shooterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterIntake.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Block Servo - keep fully open for entire autonomous
        blockServo = hardwareMap.get(Servo.class, "Block");
        blockServo.setPosition(1.0); // Fully open, no further changes needed

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter", "Ready");
        telemetry.addData("Shooter Speed Scale", "%.0f%%", SHOOTER_SPEED_SCALE * 100);
        telemetry.update();
    }

    public void start() {
        opModeTimer.resetTimer();
        setTransitionPathState(pathState);
        follower.followPath(driveStartPosShootPos, true);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }
}