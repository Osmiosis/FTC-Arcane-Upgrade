package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Robot TeleOpRed", group = "TeleOpRed")
public class TeleOpRed extends OpMode {

    // ── Drive motors ──
    private DcMotor front_left, front_right, back_left, back_right;

    // ── Slew rate limiters for smooth acceleration ──
    private final SlewRateLimiter driveLimiter  = new SlewRateLimiter(Constants.MAX_DRIVE_ACCEL);
    private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.MAX_STRAFE_ACCEL);
    private final SlewRateLimiter twistLimiter  = new SlewRateLimiter(Constants.MAX_TWIST_ACCEL);

    // ── Shooter ──
    private DcMotorEx shooterMotor1, shooterMotor2;
    private ShooterPID shooter;

    // ── Other motors ──
    private DcMotor shooterIntake;
    private DcMotor slideMotor;

    // ── AprilTag ──
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // PD controller state
    private double error = 0;
    private double lastError = 0;
    private double curTime = 0;
    private double lastTime = 0;

    // AprilTag toggle state
    private boolean aprilTagOn = false;
    private boolean aPreviouslyPressed = false;

    // Drive inputs
    private double forward, strafe, rotate;

    private FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        // ── Drive motors ──
        front_left  = hardwareMap.get(DcMotor.class, "lf");
        front_right = hardwareMap.get(DcMotor.class, "rf");
        back_left   = hardwareMap.get(DcMotor.class, "lr");
        back_right  = hardwareMap.get(DcMotor.class, "rr");

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        // ── Shooter motors ──
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor1.setDirection(DcMotorEx.Direction.REVERSE);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter = new ShooterPID(shooterMotor1, shooterMotor2);

        // ── Shooter Intake ──
        shooterIntake = hardwareMap.get(DcMotor.class, "shooter_intake");
        shooterIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── Slide ──
        slideMotor = hardwareMap.get(DcMotor.class, "slide");
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ── AprilTag + Vision Portal (MJPEG for 30 FPS, streams to Dashboard) ──
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        // Stream to FTC Dashboard
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);

        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        // Gamepad inputs
        forward = +gamepad1.left_stick_y;
        strafe  =  -gamepad1.left_stick_x;
        rotate  =  -gamepad1.right_stick_x;

        // Apply scaled deadzone
        forward = applyScaledDeadzone(forward, Constants.JOYSTICK_DEADZONE);
        strafe  = applyScaledDeadzone(strafe,  Constants.STRAFE_DEADZONE);
        rotate  = applyScaledDeadzone(rotate,  Constants.JOYSTICK_DEADZONE);

        // Apply cubic scaling for finer control
        forward = Math.copySign(Math.pow(Math.abs(forward), Constants.DRIVE_SCALE_POWER), forward);
        strafe  = Math.copySign(Math.pow(Math.abs(strafe),  Constants.DRIVE_SCALE_POWER), strafe);
        rotate  = Math.copySign(Math.pow(Math.abs(rotate),  Constants.DRIVE_SCALE_POWER), rotate);

        // Apply turn sensitivity
        rotate *= Constants.TURN_SENSITIVITY;

        // Apply slew rate limiting for smooth acceleration
        forward = driveLimiter.calculate(forward);
        strafe  = strafeLimiter.calculate(strafe);
        rotate  = twistLimiter.calculate(rotate);

        // AprilTag toggle (A button rising edge)
        boolean aPressed = gamepad1.a;
        if (aPressed && !aPreviouslyPressed) {
            aprilTagOn = !aprilTagOn;
            lastError = 0;
            lastTime = getRuntime();
        }
        aPreviouslyPressed = aPressed;

        // AprilTag detection
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection id24 = null;
        for (AprilTagDetection d : currentDetections) {
            if (d.metadata != null && d.id == 24) {
                id24 = d;
                break;
            }
        }

        // Auto-align logic (same as YouTube tutorial, bearing negated for rear-facing camera)
        if (aprilTagOn) {
            if (id24 != null) {
                // Camera faces BACKWARD → negate bearing
                error = Constants.ALIGN_GOAL_BEARING - (-id24.ftcPose.bearing);

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
            } else {
                lastTime = getRuntime();
                lastError = 0;
            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }

        // Drive call
        drive(forward, strafe, rotate);

        // ── Shooter: right trigger sets target velocity ──
        if (gamepad1.right_trigger > Constants.TRIGGER_DEADZONE) {
            shooter.setTarget(Constants.SHOOTER_TARGET);
        }
        else if (gamepad1.right_bumper){
            shooter.setTarget(Constants.SHOOTER_TARGET);
        }
        else {
            shooter.setTarget(0);
        }
        shooter.update();

        // ── Shooter Intake (hold only) ──
        if (gamepad1.left_trigger > Constants.TRIGGER_DEADZONE) {
            shooterIntake.setPower(-Constants.SHOOTER_INTAKE_POWER);
        } else if (gamepad1.left_bumper) {
            shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
        } else {
            shooterIntake.setPower(0);
        }

        // ── Slide (hold only) ──
        if (gamepad1.dpad_up) {
            slideMotor.setPower(Constants.SLIDE_POWER);
        } else if (gamepad1.dpad_down) {
            slideMotor.setPower(-Constants.SLIDE_POWER);
        } else {
            slideMotor.setPower(0);
        }

        // ── Telemetry ──
        telemetry.addData("Status", "Running");
        telemetry.addData("AprilTag Align", aprilTagOn ? "ON" : "OFF");
        if (aprilTagOn && id24 != null) {
            telemetry.addData("Tag ID", id24.id);
            telemetry.addData("Bearing", "%.1f°", id24.ftcPose.bearing);
            telemetry.addData("Error", "%.2f°", error);
            telemetry.addData("Rotate", "%.3f", rotate);
        }
        telemetry.addData("Drive", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                front_left.getPower(), front_right.getPower(),
                back_left.getPower(), back_right.getPower());

        double[] velocities = shooter.getMotorVelocities();
        telemetry.addData("Shooter", "M1:%.0f M2:%.0f", velocities[0], velocities[1]);
        telemetry.addData("Shooter Error", "%.0f", shooter.getError());
        telemetry.addData("At Target", shooter.atTarget() ? "YES" : "NO");

        String intakeStatus = shooterIntake.getPower() > 0 ? "RUNNING" :
                shooterIntake.getPower() < 0 ? "REVERSE" : "STOPPED";
        telemetry.addData("Shooter Intake", intakeStatus);

        String slideStatus = slideMotor.getPower() > 0 ? "UP" :
                slideMotor.getPower() < 0 ? "DOWN" : "STOPPED";
        telemetry.addData("Slide", slideStatus);

        telemetry.addLine();
        telemetry.addData("Controls", "LStick:Drive | RStick:Turn | A:AprilTag Toggle");
        telemetry.addData("", "RT:Shooter | LT:Intake | LB:Intake Rev");
        telemetry.addData("", "DUp:Slide Up | DDown:Slide Down");
        telemetry.update();

        // ── FTC Dashboard telemetry ──
        TelemetryPacket packet = new TelemetryPacket();
        double[] sv = shooter.getMotorVelocities();
        packet.put("Shooter Target Speed", Constants.SHOOTER_TARGET);
        packet.put("Shooter Current Speed", (sv[0] + sv[1]) / 2.0);
        packet.put("Shooter Error", shooter.getError());
        packet.put("Shooter Power Output", shooter.getOutput());
        dashboard.sendTelemetryPacket(packet);
    }

    // ── Scaled deadzone (remaps [deadzone, 1.0] → [0.0, 1.0]) ──
    private double applyScaledDeadzone(double input, double deadzone) {
        double absInput = Math.abs(input);
        if (absInput < deadzone) return 0.0;
        double scaled = (absInput - deadzone) / (1.0 - deadzone);
        return Math.copySign(scaled, input);
    }

    // ── Mecanum drive (inlined, with normalization) ──
    private void drive(double drive, double strafe, double turn) {
        double[] speeds = {
                (drive + strafe + turn),   // front_left
                (drive - strafe - turn),   // front_right
                (drive - strafe + turn),   // back_left
                (drive + strafe - turn)    // back_right
        };

        double max = 1.0;
        for (double s : speeds) {
            if (Math.abs(s) > max) max = Math.abs(s);
        }
        if (max > 1.0) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);
    }

    @Override
    public void stop() {
        FtcDashboard.getInstance().stopCameraStream();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // ══════════════════════════════════════════════════════════════════
    // SlewRateLimiter — inlined inner class, no external dependency
    // ══════════════════════════════════════════════════════════════════
    private static class SlewRateLimiter {
        private final double maxRatePerSecond;
        private double previousValue;
        private long previousTimeNanos;
        private boolean isFirstCall;

        public SlewRateLimiter(double maxRatePerSecond) {
            this.maxRatePerSecond = Math.abs(maxRatePerSecond);
            this.previousValue = 0.0;
            this.previousTimeNanos = System.nanoTime();
            this.isFirstCall = true;
        }

        public double calculate(double input) {
            long currentTimeNanos = System.nanoTime();
            if (isFirstCall) {
                previousValue = input;
                previousTimeNanos = currentTimeNanos;
                isFirstCall = false;
                return input;
            }
            double dt = (currentTimeNanos - previousTimeNanos) / 1_000_000_000.0;
            if (dt <= 0) return previousValue;

            double maxChange = maxRatePerSecond * dt;
            double desiredChange = input - previousValue;
            double limitedChange = Math.max(-maxChange, Math.min(maxChange, desiredChange));
            double output = previousValue + limitedChange;

            previousValue = output;
            previousTimeNanos = currentTimeNanos;
            return output;
        }

        public void reset() {
            previousValue = 0.0;
            previousTimeNanos = System.nanoTime();
            isFirstCall = true;
        }
    }
}