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
public class AutonomousBlueAtShoot extends OpMode {
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
    private static final double WAIT_AT_SHOOT_POSE = 2.0; // seconds to wait at shoot pose and shoot

    public enum PathState {
        //START POSITION_END POSITION
        //DRIVE > MOVEMENT STATE
        //SHOOT > ATTEMPT TO SCORE

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOSE_FIRSTCYCLEINTAKEPREP,
        INTAKE_PREP,
        DRIVE_INTAKEPREP_INTAKECOLLECT,
        INTAKE_COLLECT,
        DRIVE_INTAKECOLLECT_SHOOTPOSE,
        SHOOT_CYCLE1,
        DRIVE_SHOOTPOSE_SECONDCYCLEINTAKEPREP,
        SECOND_INTAKE_PREP,
        DRIVE_SECONDINTAKEPREP_SECONDINTAKECOLLECT,
        SECOND_INTAKE_COLLECT,
        DRIVE_SECONDINTAKECOLLECT_SHOOTPOSE,
        SHOOT_CYCLE2,
        DRIVE_SHOOTPOSE_THIRDCYCLEINTAKEPREP,
        THIRD_INTAKE_PREP,
        DRIVE_THIRDINTAKEPREP_THIRDINTAKECOLLECT,
        THIRD_INTAKE_COLLECT,
        DRIVE_THIRDINTAKECOLLECT_SHOOTPOSE,
        SHOOT_CYCLE3,
        DRIVE_TO_END_POSE,
        END
    }

    PathState pathState;

    private final Pose startPose = new Pose(20.765, 122.161, Math.toRadians(318));

    private PathChain driveStartPosShootPos;
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
        // start -> shoot (constant 318°)
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.765, 122.161), new Pose(51.612, 91.712)))
                .setConstantHeadingInterpolation(Math.toRadians(318))
                .build();

        // shoot -> first intake prep (turn 318° -> 180°)
        driveShootPosFirstCycleIntakePrep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.612, 91.712), new Pose(51.612, 83.512)))
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(180))
                .build();

        // first intake prep -> first intake collect (constant 180°)
        driveIntakePrepIntakeCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.612, 83.512), new Pose(17.813, 83.512)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // first intake collect -> shoot (turn 180° -> 318°)
        driveIntakeCollectShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(17.813, 83.512), new Pose(51.612, 91.712)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(318))
                .build();

        // shoot -> second intake prep (turn 318° -> 180°)
        driveShootPosSecondCycleIntakePrep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.612, 91.712), new Pose(51.681, 60.208)))
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(180))
                .build();

        // second intake prep -> second intake collect (constant 180°)
        driveSecondIntakePrepSecondIntakeCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.681, 60.208), new Pose(18.274, 59.861)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // second intake collect -> shoot (turn 180° -> 318°)
        driveSecondIntakeCollectShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18.274, 59.861), new Pose(51.612, 91.712)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(318))
                .build();

        // shoot -> third intake prep (turn 318° -> 180°)
        driveShootPosThirdCycleIntakePrep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.612, 91.712), new Pose(51.632, 35.640)))
                .setLinearHeadingInterpolation(Math.toRadians(318), Math.toRadians(180))
                .build();

        // third intake prep -> third intake collect (constant 180°)
        driveThirdIntakePrepThirdIntakeCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.632, 35.640), new Pose(18.967, 35.557)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // third intake collect -> shoot (turn 180° -> 318°)
        driveThirdIntakeCollectShootPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(18.967, 35.557), new Pose(51.612, 91.712)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(318))
                .build();

        // shoot -> end pose (constant 318°)
        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(51.612, 91.712), new Pose(59.061, 101.499)))
                .setConstantHeadingInterpolation(Math.toRadians(318))
                .build();
    }

    public void statePathUpdate(){
        switch (pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                // Spin up shooter while traveling to shoot position
                shooter.setTarget(Constants.SHOOTER_TARGET);
                shooter.update();
                shooterIntake.setPower(0);

                telemetry.addData("Shooter Status", "Spinning up during travel");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                telemetry.addData("Shooter Velocity", "M1:%.0f M2:%.0f",
                    shooter.getMotorVelocities()[0], shooter.getMotorVelocities()[1]);

                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                shooter.update();
                double err0 = Math.abs(shooter.getError());

                if (err0 <= 200.0 && !shootingComplete) {
                    shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                    if (shootTimer == null) {
                        shootTimer = new Timer();
                        shootTimer.resetTimer();
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
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "RUNNING" : "STOPPED");
                break;

            case DRIVE_SHOOTPOSE_FIRSTCYCLEINTAKEPREP:
                // Path already started in SHOOT_PRELOAD
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to first intake prep");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.INTAKE_PREP);
                }
                break;

            case INTAKE_PREP:
                shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "First intake prep - intake running");
                follower.followPath(driveIntakePrepIntakeCollect, true);
                setTransitionPathState(PathState.DRIVE_INTAKEPREP_INTAKECOLLECT);
                break;

            case DRIVE_INTAKEPREP_INTAKECOLLECT:
                shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Driving to first intake collect");
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
                shooter.setTarget(Constants.SHOOTER_TARGET);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Returning to shooter - spinning up");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_CYCLE1);
                }
                break;

            case SHOOT_CYCLE1:
                shooter.update();
                double err1 = Math.abs(shooter.getError());

                if (err1 <= 200.0 && !shootingComplete2) {
                    shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                    if (shootTimer2 == null) {
                        shootTimer2 = new Timer();
                        shootTimer2.resetTimer();
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
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "RUNNING" : "STOPPED");
                break;

            case DRIVE_SHOOTPOSE_SECONDCYCLEINTAKEPREP:
                // Path already started in SHOOT_CYCLE1
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to second intake prep");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SECOND_INTAKE_PREP);
                }
                break;

            case SECOND_INTAKE_PREP:
                shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Second intake prep - intake running");
                follower.followPath(driveSecondIntakePrepSecondIntakeCollect, true);
                setTransitionPathState(PathState.DRIVE_SECONDINTAKEPREP_SECONDINTAKECOLLECT);
                break;

            case DRIVE_SECONDINTAKEPREP_SECONDINTAKECOLLECT:
                shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Driving to second intake collect");
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
                shooter.setTarget(Constants.SHOOTER_TARGET);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Returning to shooter (cycle2) - spinning up");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_CYCLE2);
                }
                break;

            case SHOOT_CYCLE2:
                shooter.update();
                double err2 = Math.abs(shooter.getError());

                if (err2 <= 200.0 && !shootingComplete3) {
                    shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                    if (shootTimer3 == null) {
                        shootTimer3 = new Timer();
                        shootTimer3.resetTimer();
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
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "RUNNING" : "STOPPED");
                break;

            case DRIVE_SHOOTPOSE_THIRDCYCLEINTAKEPREP:
                // Path already started in SHOOT_CYCLE2
                shooter.setTarget(0);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Driving to third intake prep");
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.THIRD_INTAKE_PREP);
                }
                break;

            case THIRD_INTAKE_PREP:
                shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Third intake prep - intake running");
                follower.followPath(driveThirdIntakePrepThirdIntakeCollect, true);
                setTransitionPathState(PathState.DRIVE_THIRDINTAKEPREP_THIRDINTAKECOLLECT);
                break;

            case DRIVE_THIRDINTAKEPREP_THIRDINTAKECOLLECT:
                shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                shooter.setTarget(0);
                shooter.update();
                telemetry.addData("Status", "Driving to third intake collect");
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
                shooter.setTarget(Constants.SHOOTER_TARGET);
                shooter.update();
                shooterIntake.setPower(0);
                telemetry.addData("Status", "Returning to shooter (cycle3) - spinning up");
                telemetry.addData("Shooter Error", "%.0f", Math.abs(shooter.getError()));
                if (!follower.isBusy()) {
                    setTransitionPathState(PathState.SHOOT_CYCLE3);
                }
                break;

            case SHOOT_CYCLE3:
                shooter.update();
                double err3 = Math.abs(shooter.getError());
                if (err3 <= 200.0) {
                    shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
                    if (shootTimer3 == null) {
                        shootTimer3 = new Timer();
                        shootTimer3.resetTimer();
                    }
                    if (shootTimer3.getElapsedTimeSeconds() >= WAIT_AT_SHOOT_POSE) {
                        shooterIntake.setPower(0);
                        shooter.setTarget(0);
                        shooter.update();
                        follower.followPath(driveShootPosEndPose, true);
                        setTransitionPathState(PathState.DRIVE_TO_END_POSE);
                    } else {
                        telemetry.addData("Cycle3 Timer", "%.1f / %.1f", shootTimer3.getElapsedTimeSeconds(), WAIT_AT_SHOOT_POSE);
                    }
                } else {
                    shooterIntake.setPower(0);
                    telemetry.addData("Shooter Status", "Waiting for speed");
                }
                telemetry.addData("Shooter Error", "%.0f", err3);
                telemetry.addData("Shooter Intake", shooterIntake.getPower() > 0 ? "RUNNING" : "STOPPED");
                break;

            case DRIVE_TO_END_POSE:
                // Path already started in SHOOT_CYCLE3
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
        shooterIntake.setDirection(DcMotor.Direction.REVERSE); // change for now

        // Initialize Block Servo - keep fully open for entire autonomous
        blockServo = hardwareMap.get(Servo.class, "Block");
        blockServo.setPosition(1.0); // Fully open, no further changes needed

        buildPaths();
        follower.setPose(startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Shooter", "Ready");
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
