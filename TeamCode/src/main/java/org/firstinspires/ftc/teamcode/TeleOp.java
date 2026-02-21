package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Robot TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    // ── Subsystems ──
    private final AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private MecanumDrive drive;
    private ShooterPID shooter;
    private FtcDashboard dashboard;

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private DcMotor shooterIntake;
    private DcMotor slideMotor;

    // ── AprilTag PD controller variables (matches YouTube tutorial) ──
    private double error = 0;
    private double lastError = 0;
    private double curTime = 0;
    private double lastTime = 0;

    // AprilTag toggle state
    private boolean aprilTagOn = false;
    private boolean aPreviouslyPressed = false;

    // Drive inputs
    private double forward, strafe, rotate;

    @Override
    public void init() {
        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize AprilTag webcam (streams to FTC Dashboard automatically)
        aprilTagWebcam.init(hardwareMap, telemetry);

        // Initialize Mecanum Drive
        drive = new MecanumDrive(hardwareMap);

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

        // Initialize Slide Motor
        slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive", "Ready");
        telemetry.addData("Shooter", "Ready");
        telemetry.addData("Shooter Intake", "Ready");
        telemetry.addData("Slide", "Ready");
        telemetry.addData("AprilTag", "Ready (A to toggle align)");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // ── Gamepad inputs ──
        forward = -gamepad1.left_stick_y;
        strafe  =  gamepad1.left_stick_x;
        rotate  =  gamepad1.right_stick_x;

        // ── AprilTag toggle (A button rising edge) ──
        boolean aPressed = gamepad1.a;
        if (aPressed && !aPreviouslyPressed) {
            aprilTagOn = !aprilTagOn;
            lastError = 0;
            lastTime = getRuntime();
        }
        aPreviouslyPressed = aPressed;

        // ── AprilTag detection (always update so dashboard shows the feed) ──
        aprilTagWebcam.update();

        // ── Auto-align logic (copied from YouTube tutorial, with rear-camera negate) ──
        if (aprilTagOn) {
            // Get first visible tag (or use getTagBySpecificId(20) for a specific one)
            AprilTagDetection target = aprilTagWebcam.getFirstDetection();

            if (target != null) {
                // Camera faces BACKWARD → negate bearing so robot turns correctly
                error = Constants.ALIGN_GOAL_BEARING - (-target.ftcPose.bearing);

                if (Math.abs(error) < Constants.ALIGN_ANGLE_TOLERANCE) {
                    rotate = 0;
                } else {
                    double pTerm = error * Constants.ALIGN_KP;
                    curTime = getRuntime();
                    double dt = curTime - lastTime;
                    double dTerm = ((error - lastError) / dt) * Constants.ALIGN_KD;

                    rotate = Range.clip(pTerm + dTerm, -Constants.ALIGN_MAX_ROTATE, Constants.ALIGN_MAX_ROTATE);

                    lastError = error;
                    lastTime = curTime;
                }

                telemetry.addData("AprilTag", "Aligning  ID:%d  Bearing:%.1f°  Error:%.2f°",
                        target.id, target.ftcPose.bearing, error);
                telemetry.addData("Align Rotate", "%.3f", rotate);
            } else {
                lastTime = getRuntime();
                lastError = 0;
                telemetry.addData("AprilTag", "Searching... (no tag detected)");
            }

            // Use setRobotCentric during alignment (bypasses slew rate limiting for precise control)
            drive.setRobotCentric(forward, strafe, rotate);
        } else {
            lastError = 0;
            lastTime = getRuntime();

            // Normal manual drive — all sticks active with slew rate limiting
            drive.update(gamepad1);
        }

        // ── Shooter: right trigger sets target velocity ──
        if (gamepad1.right_trigger > Constants.TRIGGER_DEADZONE) {
            shooter.setTarget(Constants.SHOOTER_TARGET);
        } else {
            shooter.setTarget(0);
        }
        shooter.update();

        // ── Shooter Intake (hold only) ──
        // Left Trigger: run forward | Left Bumper: run in reverse
        if (gamepad1.left_trigger > Constants.TRIGGER_DEADZONE) {
            shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
        } else if (gamepad1.left_bumper) {
            shooterIntake.setPower(-Constants.SHOOTER_INTAKE_POWER);
        } else {
            shooterIntake.setPower(0);
        }

        // ── Slide (hold only) ──
        // DPad Up: extend | DPad Down: retract
        if (gamepad1.dpad_up) {
            slideMotor.setPower(Constants.SLIDE_POWER);
        } else if (gamepad1.dpad_down) {
            slideMotor.setPower(-Constants.SLIDE_POWER);
        } else {
            slideMotor.setPower(0);
        }

        // ── Telemetry ──
        double[] motorPowers = drive.getMotorPowers();
        telemetry.addData("Status", "Running");
        telemetry.addLine();

        telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);

        telemetry.addLine();
        double[] velocities = shooter.getMotorVelocities();
        telemetry.addData("Shooter Target", "%.0f ticks/sec", Constants.SHOOTER_TARGET);
        telemetry.addData("Shooter Velocities", "M1:%.0f M2:%.0f", velocities[0], velocities[1]);
        telemetry.addData("Shooter Error", "%.0f", shooter.getError());
        telemetry.addData("Shooter Power", "%.3f", shooter.getOutput());
        telemetry.addData("At Target", shooter.atTarget() ? "YES" : "NO");

        telemetry.addLine();
        boolean shooterReady = Math.abs(shooter.getError()) <= Constants.SHOOTER_READY_TOLERANCE;
        telemetry.addData("Shooter Ready", shooterReady ? "YES" : "NO");
        String shooterIntakeStatus = shooterIntake.getPower() > 0 ? "RUNNING" :
                shooterIntake.getPower() < 0 ? "REVERSE" : "STOPPED";
        telemetry.addData("Shooter Intake", shooterIntakeStatus);

        telemetry.addLine();
        String slideStatus = slideMotor.getPower() > 0 ? "UP" :
                slideMotor.getPower() < 0 ? "DOWN" : "STOPPED";
        telemetry.addData("Slide", slideStatus);

        telemetry.addLine();
        telemetry.addData("Controls", "Left Stick: Drive/Strafe | Right Stick: Turn");
        telemetry.addData("AprilTag Align", "A: Toggle align (ON: " + (aprilTagOn ? "YES" : "NO") + ")");
        telemetry.addData("Shooter", "Right Trigger: Activate");
        telemetry.addData("Shooter Intake", "Left Trigger: Feed | Left Bumper: REVERSE");
        telemetry.addData("Slide", "DPad Up: Extend | DPad Down: Retract");

        telemetry.update();

        // ── FTC Dashboard telemetry ──
        TelemetryPacket packet = new TelemetryPacket();
        double[] shooterVelocities = shooter.getMotorVelocities();
        double currentSpeed = (shooterVelocities[0] + shooterVelocities[1]) / 2.0;

        packet.put("Shooter Target Speed", Constants.SHOOTER_TARGET);
        packet.put("Shooter Current Speed", currentSpeed);
        packet.put("Shooter Motor 1 Speed", shooterVelocities[0]);
        packet.put("Shooter Motor 2 Speed", shooterVelocities[1]);
        packet.put("Shooter Error", shooter.getError());
        packet.put("Shooter Power Output", shooter.getOutput());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        aprilTagWebcam.close();
    }
}