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
public class AutonomousBlueSubmissive extends OpMode {
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
    private static final double WAIT_AT_SHOOT_POSE = 2.5;

    // ── Intake pulse settings ──────────────────────────────────────────────────
    private static final double SHOOT_PULSE_CYCLE = WAIT_AT_SHOOT_POSE / 3.0;
    private static final double SHOOT_PULSE_ON    = SHOOT_PULSE_CYCLE / 2.0;

    private Timer pulseTimer = new Timer();

    // ── Shooter speed ──────────────────────────────────────────────────────────
    private static final double SHOOTER_SPEED_SCALE = 0.83;

    /** Pulsed power while SHOOTING — exactly 3 pulses over WAIT_AT_SHOOT_POSE seconds. */
    private double getShootingPulsePower() {
        double elapsed = pulseTimer.getElapsedTimeSeconds() % SHOOT_PULSE_CYCLE;
        return elapsed < SHOOT_PULSE_ON ? Constants.SHOOTER_INTAKE_POWER : 0.0;
    }

    private void startIntakePulse() {
        pulseTimer.resetTimer();
    }

    private double scaledShooterTarget() {
        return Constants.SHOOTER_TARGET * SHOOTER_SPEED_SCALE;
    }

    public enum PathState {
        DRIVE_STARTPOS_SHOOT_POS,    // drive up at 90°
        ALIGN_TO_CITADEL,            // nudge down, rotate 90° -> 136°
        SHOOT_PRELOAD,               // shoot at 136°
        DRIVE_TO_END_POSE,           // drive to end, rotate 136° -> 90°
        END
    }

    PathState pathState;

    private final Pose startPose = new Pose(60.199, 8.003, Math.toRadians(90));

    private PathChain driveStartPosShootPos;
    private PathChain alignToCitadel;
    private PathChain driveToEndPose;

    public void buildPaths() {
        // start -> shoot approach (constant 90°) — drive straight up
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.199, 8.003), new Pose(59.762, 92.233)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        // nudge down and rotate to shooting heading (90° -> 136°)
        alignToCitadel = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.762, 92.233), new Pose(60.053, 83.210)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(136))
                .build();

        // after shooting, drive to end pose and rotate back (136° -> 90°)
        driveToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.053, 83.210), new Pose(68.164, 38.899)))
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(90))
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                // Spinner up while driving up
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
                // Nudge down while rotating to 136° — shooter stays spun up
                shooter.setTarget(scaledShooterTarget());
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Aligning to citadel (rotating to 136°)...");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                // Shoot at 136° heading with pulsed intake
                shooter.update();
                double err0 = Math.abs(shooter.getError());
                if (err0 <= 200.0 && !shootingComplete) {
                    shooterIntake.setPower(getShootingPulsePower());
                    if (shootTimer == null) {
                        shootTimer = new Timer();
                        shootTimer.resetTimer();
                        startIntakePulse();
                    }
                    if (shootTimer.getElapsedTimeSeconds() >= WAIT_AT_SHOOT_POSE) {
                        shooterIntake.setPower(0);
                        shooter.setTarget(0);
                        shooter.update();
                        shootingComplete = true;
                        follower.followPath(driveToEndPose, true);
                        setTransitionPathState(PathState.DRIVE_TO_END_POSE);
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

            case DRIVE_TO_END_POSE:
                // Drive to end position, rotating back to 90°
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to end pose (rotating to 90°)");
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

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter = new ShooterPID(shooterMotor1, shooterMotor2);

        shooterIntake = hardwareMap.get(DcMotor.class, "shooter_intake");
        shooterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterIntake.setDirection(DcMotor.Direction.REVERSE);

        blockServo = hardwareMap.get(Servo.class, "Block");
        blockServo.setPosition(1.0);

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