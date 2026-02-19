package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Robot TeleOp", group = "TeleOp")
public class TeleOp extends OpMode {

    private MecanumDrive drive;
    private ShooterPID shooter;
    private FtcDashboard dashboard;

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private DcMotor shooterIntake;
    private Servo blockServo;

    // Toggle state for main intake
    private boolean mainIntakeOn = false;
    private boolean xButtonPreviouslyPressed = false;

    // AprilTag alignment (LB held)
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private boolean lbPreviouslyPressed = false;

    // Camera offset from the shooter centerline (fill in after measuring)
    private static final double CAMERA_OFFSET_X = 0.0; // inches, positive = camera is to the right of shooter

    // Alignment gains — tune these for your robot
    private static final double TURN_GAIN   = 0.02;  // turn power per degree of bearing error
    private static final double STRAFE_GAIN = 0.02;  // strafe power per degree of yaw error
    private static final double MAX_TURN    = 0.4;
    private static final double MAX_STRAFE  = 0.4;


    @Override
    public void init() {
        // Initialize FTC Dashboard for graphing
        dashboard = FtcDashboard.getInstance();

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

        // Initialize Block Servo
        blockServo = hardwareMap.get(Servo.class, "Block");
        blockServo.setPosition(0); // Start in closed position

        // Initialize AprilTag processor (camera faces backward with shooter)
        // C920 has built-in SDK calibration at 800x600 — no manual calibration needed
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(800, 600))
                .addProcessor(aprilTag)
                .build();
        // Wait for camera to finish opening, then suspend to save CPU until LB is pressed
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            // yield — init() blocks here briefly until camera is ready
        }
        visionPortal.stopStreaming();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive", "Ready");
        telemetry.addData("Shooter", "Ready");
        telemetry.addData("Shooter Intake", "Ready");
        telemetry.addData("Main Intake", "Ready");
        telemetry.addData("Climb", "Ready");
        telemetry.addData("AprilTag", "Ready (LB to align)");
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        // ── AprilTag alignment (LB) ──────────────────────────────────────────
        boolean lbHeld = gamepad1.left_bumper;

        // Start/stop camera stream on LB edge to save CPU
        if (lbHeld && !lbPreviouslyPressed) {
            visionPortal.resumeStreaming();
        } else if (!lbHeld && lbPreviouslyPressed) {
            visionPortal.stopStreaming();
            drive.stop(); // clear any alignment powers and reset slew limiters
        }
        lbPreviouslyPressed = lbHeld;

        if (lbHeld) {
            // --- AprilTag alignment mode: TURN + STRAFE to face the tag ---
            // Goal: rotate the robot so the shooter (rear) faces the tag,
            //       then strafe to put the shooter centerline directly on the tag.
            // No forward/backward drive — we never approach the tag.
            AprilTagDetection target = getBestTag();

            if (target != null) {
                // bearing: degrees left(+) or right(-) the tag is from the camera's center axis.
                // Camera faces BACKWARD, so a positive bearing (tag left of camera) means
                // the tag is actually to the RIGHT of the shooter → robot must turn RIGHT (positive turn).
                double bearingError = target.ftcPose.bearing; // degrees
                double rangeInches  = target.ftcPose.range;   // inches

                // Apply camera offset correction: convert physical X/Y offset (inches) to
                // an equivalent angular offset at this range, then subtract it out so the
                // shooter centerline (not the lens) is what aligns to the tag.
                double offsetBearingCorrection = (rangeInches > 0)
                        ? Math.toDegrees(Math.atan2(CAMERA_OFFSET_X, rangeInches)) : 0;
                double correctedBearing = bearingError - offsetBearingCorrection;

                // Turn: camera is backward → negate so positive bearing turns the shooter toward tag.
                // Strafe: lateral offset of tag = range * sin(bearing). Correct for camera X offset too.
                double lateralOffset = rangeInches * Math.sin(Math.toRadians(correctedBearing)) - CAMERA_OFFSET_X;

                double turn   = Math.max(-MAX_TURN,   Math.min(MAX_TURN,    correctedBearing * TURN_GAIN));
                double strafe = Math.max(-MAX_STRAFE,  Math.min(MAX_STRAFE, -lateralOffset    * STRAFE_GAIN));

                drive.setRobotCentric(0, strafe, turn);

                telemetry.addData("AprilTag", "Aligning  ID:%d  Bearing:%.1f°  Lateral:%.2f\"",
                        target.id, correctedBearing, lateralOffset);
                telemetry.addData("Align Powers", "Turn:%.3f  Strafe:%.3f", turn, strafe);
            } else {
                // No tag visible — hold still, keep trying
                drive.stop();
                telemetry.addData("AprilTag", "Searching... (no tag detected)");
            }
        } else {
            // Normal manual drive — all sticks active
            drive.update(gamepad1);
        }
        // ─────────────────────────────────────────────────────────────────────

        // Update Shooter: trigger sets target velocity, PID always runs
        if (gamepad1.right_trigger > Constants.TRIGGER_DEADZONE) {
            shooter.setTarget(Constants.SHOOTER_TARGET);
        } else {
            shooter.setTarget(0);  // Stop shooter when trigger released
        }
        shooter.update();

        // Block Servo Control - Only opens when trigger is held AND shooter is within tolerance of target speed
        boolean shooterTriggered = gamepad1.right_trigger > Constants.TRIGGER_DEADZONE;
        double shooterError = Math.abs(shooter.getError());
        if (shooterTriggered && shooterError <= Constants.SHOOTER_READY_TOLERANCE) {
            blockServo.setPosition(1); // Open
        } else {
            blockServo.setPosition(0); // Closed
        }

        // Shooter Intake Control - Only runs when dpad_up is held AND shooter is at speed
        // Emergency reverse with dpad_down
        boolean shooterReady = Math.abs(shooter.getError()) <= Constants.SHOOTER_READY_TOLERANCE;
        if (gamepad1.dpad_up && shooterReady) {
            shooterIntake.setPower(Constants.SHOOTER_INTAKE_POWER);
        } else if (gamepad1.dpad_down) {
            // Emergency reverse - no shooter ready check
            shooterIntake.setPower(-Constants.SHOOTER_INTAKE_POWER);
        } else {
            shooterIntake.setPower(0);
        }

        // Main Intake Control - Toggle with X button, Emergency reverse with B
        // Detect X button press (edge detection)
        if (gamepad1.x && !xButtonPreviouslyPressed) {
            mainIntakeOn = !mainIntakeOn;  // Toggle state
        }
        xButtonPreviouslyPressed = gamepad1.x;

        // Apply power based on state and emergency override
        if (gamepad1.b) {
            // Emergency reverse overrides everything
            //mainIntake.setPower(-Constants.MAIN_INTAKE_POWER);
        } else if (mainIntakeOn) {
            // Normal operation when toggled on
            //mainIntake.setPower(Constants.MAIN_INTAKE_POWER);
        } else {
            // Off
            //mainIntake.setPower(0);
        }

        // Climb Control - Y button (up), A button (down)
        if (gamepad1.y) {
            //climb.setPower(Constants.CLIMB_POWER);
        } else if (gamepad1.a) {
            //climb.setPower(-Constants.CLIMB_POWER);
        } else {
        }

        // Display Drive Telemetry
        double[] motorPowers = drive.getMotorPowers();
        telemetry.addData("Status", "Running");
        telemetry.addLine();

        telemetry.addData("Drive Motors", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);

        // Display Shooter Telemetry
        telemetry.addLine();
        double[] velocities = shooter.getMotorVelocities();
        telemetry.addData("Shooter Target", "%.0f ticks/sec", Constants.SHOOTER_TARGET);
        telemetry.addData("Shooter Velocities", "M1:%.0f M2:%.0f", velocities[0], velocities[1]);
        telemetry.addData("Shooter Error", "%.0f", shooter.getError());
        telemetry.addData("Shooter Power", "%.3f", shooter.getOutput());
        telemetry.addData("At Target", shooter.atTarget() ? "YES" : "NO");
        telemetry.addData("Block Servo", blockServo.getPosition() == 1 ? "OPEN" : "CLOSED");

        // Display Shooter Intake Telemetry
        telemetry.addLine();
        telemetry.addData("Shooter Ready", shooterReady ? "YES" : "NO");
        String shooterIntakeStatus = shooterIntake.getPower() > 0 ? "RUNNING" :
                             shooterIntake.getPower() < 0 ? "REVERSE" : "STOPPED";
        telemetry.addData("Shooter Intake", shooterIntakeStatus);

        // Display Main Intake Telemetry
        telemetry.addLine();
        //String mainIntakeStatus = mainIntake.getPower() > 0 ? "ON" :
                                 //mainIntake.getPower() < 0 ? "REVERSE" : "OFF";
        //telemetry.addData("Main Intake", mainIntakeStatus);
        telemetry.addData("Intake Toggle State", mainIntakeOn ? "ON" : "OFF");

        // Display Climb Telemetry
        telemetry.addLine();
        //String climbStatus = climb.getPower() > 0 ? "UP" :
                            //climb.getPower() < 0 ? "DOWN" : "STOPPED";
        //telemetry.addData("Climb", climbStatus);

        // Display Controls
        telemetry.addLine();
        telemetry.addData("Controls", "Left Stick: Drive/Strafe | Right Stick: Turn");
        telemetry.addData("AprilTag Align", "LB: Hold to align shooter to tag");
        telemetry.addData("Shooter", "Right Trigger: Activate");
        telemetry.addData("Shooter Intake", "DPad Up: Feed | DPad Down: REVERSE");
        telemetry.addData("Main Intake", "X: Toggle ON/OFF | B: REVERSE");
        telemetry.addData("Climb", "Y: UP | A: DOWN");

        telemetry.update();

        // Send data to FTC Dashboard for graphing
        TelemetryPacket packet = new TelemetryPacket();
        double[] shooterVelocities = shooter.getMotorVelocities();
        double currentSpeed = (shooterVelocities[0] + shooterVelocities[1]) / 2.0;

        packet.put("Shooter Target Speed", Constants.SHOOTER_TARGET);
        packet.put("Shooter Current Speed", currentSpeed);
        packet.put("Shooter Motor 1 Speed", shooterVelocities[0]);
        packet.put("Shooter Motor 2 Speed", shooterVelocities[1]);
        packet.put("Shooter Error", shooter.getError());
        packet.put("Shooter Power Output", shooter.getOutput());
        packet.put("Block Servo Position", blockServo.getPosition());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        visionPortal.close();
    }

    /**
     * Returns the best (closest) AprilTag detection, or null if none found.
     */
    private AprilTagDetection getBestTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        AprilTagDetection best = null;
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) {
                if (best == null || d.ftcPose.range < best.ftcPose.range) {
                    best = d;
                }
            }
        }
        return best;
    }

    // ========================== INNER CLASSES ==========================

    /**
     * SlewRateLimiter - Limits the rate of change of a value for smooth acceleration
     */
    private static class SlewRateLimiter {
        private final double maxRatePerSecond; // Maximum allowed change per second
        private double previousValue;          // Last output value
        private long previousTimeNanos;        // Last calculation time in nanoseconds
        private boolean isFirstCall;           // Track if this is the first calculation

        public SlewRateLimiter(double maxRatePerSecond) {
            this.maxRatePerSecond = Math.abs(maxRatePerSecond); // Ensure positive
            this.previousValue = 0.0;
            this.previousTimeNanos = System.nanoTime();
            this.isFirstCall = true;
        }

        public double calculate(double input) {
            // Get current time
            long currentTimeNanos = System.nanoTime();

            // On first call, just return the input and initialize
            if (isFirstCall) {
                previousValue = input;
                previousTimeNanos = currentTimeNanos;
                isFirstCall = false;
                return input;
            }

            // Calculate time elapsed in seconds
            double deltaTimeSeconds = (currentTimeNanos - previousTimeNanos) / 1_000_000_000.0;

            // Handle edge cases
            if (deltaTimeSeconds <= 0) {
                // No time has passed, return previous value
                return previousValue;
            }

            // Calculate maximum allowed change for this time step
            double maxChange = maxRatePerSecond * deltaTimeSeconds;

            // Calculate desired change
            double desiredChange = input - previousValue;

            // Clamp the change to the maximum allowed
            double limitedChange = Math.max(-maxChange, Math.min(maxChange, desiredChange));

            // Calculate new output value
            double output = previousValue + limitedChange;

            // Update state for next call
            previousValue = output;
            previousTimeNanos = currentTimeNanos;

            return output;
        }

        public void reset() {
            previousValue = 0.0;
            previousTimeNanos = System.nanoTime();
            isFirstCall = true;
        }

        public void reset(double value) {
            previousValue = value;
            previousTimeNanos = System.nanoTime();
            isFirstCall = false;
        }

        public double getLastValue() {
            return previousValue;
        }
    }

    /**
     * MecanumDrive - Handles mecanum drive control with slew rate limiting
     */
    private static class MecanumDrive {
        private DcMotor front_left  = null;
        private DcMotor front_right = null;
        private DcMotor back_left   = null;
        private DcMotor back_right  = null;

        // Slew rate limiters for smooth acceleration
        private final SlewRateLimiter driveLimiter = new SlewRateLimiter(Constants.MAX_DRIVE_ACCEL);
        private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(Constants.MAX_STRAFE_ACCEL);
        private final SlewRateLimiter twistLimiter = new SlewRateLimiter(Constants.MAX_TWIST_ACCEL);

        // Current motor powers
        private double[] currentSpeeds = new double[4];

        public MecanumDrive(HardwareMap hardwareMap) {
            front_left = hardwareMap.get(DcMotor.class, "lf");
            front_right = hardwareMap.get(DcMotor.class, "rf");
            back_left = hardwareMap.get(DcMotor.class, "lr");
            back_right = hardwareMap.get(DcMotor.class, "rr");

            front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            front_left.setDirection(DcMotor.Direction.REVERSE);
            back_left.setDirection(DcMotor.Direction.REVERSE);

            front_right.setDirection(DcMotor.Direction.FORWARD);
            back_right.setDirection(DcMotor.Direction.FORWARD);
        }

        private double applyScaledDeadzone(double input, double deadzone) {
            double absInput = Math.abs(input);

            // If within deadzone, return zero
            if (absInput < deadzone) {
                return 0.0;
            }

            // Remap [deadzone, 1.0] to [0.0, 1.0] and preserve sign
            double scaled = (absInput - deadzone) / (1.0 - deadzone);
            return Math.copySign(scaled, input);
        }

        public void update(Gamepad gamepad) {
            double drive  = -gamepad.left_stick_y;
            double strafe = +gamepad.left_stick_x;
            double twist  = -gamepad.right_stick_x;

            // Apply scaled deadzone to ignore small stick movements while preserving smooth control
            // This remaps the range [DEADZONE, 1.0] to [0.0, 1.0] to avoid jumps
            drive = applyScaledDeadzone(drive, Constants.JOYSTICK_DEADZONE);
            strafe = applyScaledDeadzone(strafe, Constants.JOYSTICK_DEADZONE);
            twist = applyScaledDeadzone(twist, Constants.JOYSTICK_DEADZONE);

            // Apply non-linear scaling for finer control (preserve sign)
            drive = Math.copySign(Math.pow(Math.abs(drive), Constants.DRIVE_SCALE_POWER), drive);
            strafe = Math.copySign(Math.pow(Math.abs(strafe), Constants.DRIVE_SCALE_POWER), strafe);
            twist = Math.copySign(Math.pow(Math.abs(twist), Constants.DRIVE_SCALE_POWER), twist);

            // Apply slew rate limiting for smooth acceleration
            drive = driveLimiter.calculate(drive);
            strafe = strafeLimiter.calculate(strafe);
            twist = twistLimiter.calculate(twist);

            // Calculate mecanum wheel speeds
            double[] speeds = {
                    (drive + strafe + twist),  // front_left
                    (drive - strafe - twist),  // front_right
                    (drive - strafe + twist),  // back_left
                    (drive + strafe - twist)   // back_right
            };

            applyAndSetPowers(speeds);
        }

        public double[] getMotorPowers() {
            return currentSpeeds;
        }

        public void stop() {
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

            // Reset slew rate limiters to prevent jumps
            driveLimiter.reset();
            strafeLimiter.reset();
            twistLimiter.reset();
        }

        public void setRobotCentric(double drive, double strafe, double turn) {
            // Calculate mecanum wheel speeds
            double[] speeds = {
                    (drive + strafe + turn),  // front_left
                    (drive - strafe - turn),  // front_right
                    (drive - strafe + turn),  // back_left
                    (drive + strafe - turn)   // back_right
            };

            applyAndSetPowers(speeds);
        }

        private void applyAndSetPowers(double[] speeds) {
            // Find the maximum absolute value of the speeds
            double max = 1.0;
            for(int i = 0; i < speeds.length; i++) {
                if (Math.abs(speeds[i]) > max) {
                    max = Math.abs(speeds[i]);
                }
            }

            // Normalize the speeds if any is greater than 1.
            if (max > 1.0) {
                for (int i = 0; i < speeds.length; i++) {
                    speeds[i] /= max;
                }
            }

            // Store current speeds for telemetry (create a copy to avoid reference issues)
            currentSpeeds = Arrays.copyOf(speeds, 4);

            // Apply the calculated values to the motors
            front_left.setPower(speeds[0]);
            front_right.setPower(speeds[1]);
            back_left.setPower(speeds[2]);
            back_right.setPower(speeds[3]);
        }
    }

    /**
     * ShooterPID - PID controller for shooter motors
     */
    private static class ShooterPID {
        private DcMotorEx shooterMotor1;
        private DcMotorEx shooterMotor2;
        private ElapsedTime timerS;

        private double lastShooterError = 0;
        private double shoot = 0;
        private double shooterTarget = Constants.SHOOTER_TARGET;
        private boolean targetChanged = false;

        public ShooterPID(DcMotorEx motor1, DcMotorEx motor2) {
            this.shooterMotor1 = motor1;
            this.shooterMotor2 = motor2;
            this.timerS = new ElapsedTime();
            timerS.reset();
        }

        public void setTarget(double target) {
            if (Math.abs(this.shooterTarget - target) > 0.1) {
                targetChanged = true;
            }
            this.shooterTarget = target;
        }

        private void calculatePID() {
            // Always calculate PID
            double currentVelocity = (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
            double error = shooterTarget - currentVelocity;

            // Calculate dt using milliseconds (more stable than microseconds)
            double dt = Math.max(timerS.milliseconds() / 1000.0, 1e-3);

            // Calculate derivative, but prevent derivative kick on target changes
            double derivative = targetChanged ? 0 : (error - lastShooterError) / dt;
            targetChanged = false;

            // Feedforward (scaled properly to motor max RPM)
            double maxRPM = 6000;
            shoot = Constants.Kp_Shoot * error + Constants.Kd_shoot * derivative + Constants.Kf_Shoot * (shooterTarget / maxRPM);

            // Clamp power to ±1 to prevent SDK overhead and jitter
            shoot = Math.max(-1.0, Math.min(shoot, 1.0));

            lastShooterError = error;

            // Reset timer at the end for next cycle
            timerS.reset();

            // Apply power to both motors
            shooterMotor1.setPower(shoot);
            shooterMotor2.setPower(shoot);
        }

        public double update() {
            // PID always runs
            calculatePID();
            return shoot;
        }

        public double updateAuto() {
            calculatePID();
            return shoot;
        }

        public double getOutput() {
            return shoot;
        }

        public double getError() {
            double avgVelocity = (shooterMotor1.getVelocity() + shooterMotor2.getVelocity()) / 2.0;
            return shooterTarget - avgVelocity;
        }

        public double[] getMotorVelocities() {
            return new double[]{shooterMotor1.getVelocity(), shooterMotor2.getVelocity()};
        }

        public boolean atTarget() {
            return Math.abs(getError()) <= Constants.Shooter_Tolerance;
        }

        public void reset() {
            lastShooterError = 0;
            shoot = 0;
            timerS.reset();
        }

        public void stop() {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
            reset();
        }
    }
}
