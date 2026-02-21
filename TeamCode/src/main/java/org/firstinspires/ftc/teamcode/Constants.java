package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // Mecanum Drive Configuration
    public static double JOYSTICK_DEADZONE = 0.05;        // Ignore small stick movements below this threshold
    public static double DRIVE_SCALE_POWER = 3.0;         // Non-linear scaling power (1.0 = linear, 3.0 = cubic)

    // Slew Rate Limiting (units per second) - prevents jerky movements and motor strain
    public static double MAX_DRIVE_ACCEL = 3.0;           // Max drive acceleration (units/sec) - 0 to full in 0.33s
    public static double MAX_STRAFE_ACCEL = 3.0;          // Max strafe acceleration (units/sec) - 0 to full in 0.33s
    public static double MAX_TWIST_ACCEL = 4.0;           // Max rotation acceleration (units/sec) - 0 to full in 0.25s (faster for quick turns)
    public static double Shooter_Tolerance = 200.0;        // Tolerance for shooter error (within this range = on target)
    public static double TRIGGER_DEADZONE = 0.05;         // Right trigger activation threshold
    public static double SHOOTER_TARGET = 1700.0;         // Target shooter velocity in ticks per second

    // Shooter PID Constants
    public static double Kp_Shoot = 0.001;                // Proportional gain for shooter
    public static double Kd_shoot = 0;               // Derivative gain for shooter
    public static double Kf_Shoot = 2.3;               // Feedforward gain for shooter

    // Shooter Intake Configuration
    public static double SHOOTER_INTAKE_POWER = 1.0;      // Power for shooter intake motor
    public static double SHOOTER_READY_TOLERANCE = 40.0;  // Shooter must be within this tolerance to activate intake/open block servo

    // Main Intake Configuration
    public static double MAIN_INTAKE_POWER = 1.0;         // Power for main intake motor

    // Climb Configuration
    public static double CLIMB_POWER = 1.0;               // Power for climb motor

    // Slide Configuration
    public static double SLIDE_POWER = 1.0;               // Power for slide motor (dpad up/down)

    // AprilTag Auto-Align (PD controller, activated by LB)
    public static double ALIGN_KP = 0.002;                // Proportional gain for bearing alignment
    public static double ALIGN_KD = 0.0001;               // Derivative gain for bearing alignment
    public static double ALIGN_GOAL_BEARING = 0.0;        // Target bearing (degrees) — 0 = tag straight ahead of shooter
    public static double ALIGN_ANGLE_TOLERANCE = 0.2;     // Bearing tolerance (degrees) before stopping rotation
    public static double ALIGN_MAX_ROTATE = 0.4;          // Max rotate power during alignment

    // Deprecated/Removed: AprilTag Localization Configuration
    // These values are preserved only as reference; AprilTag/webcam usage has been removed.
    @Deprecated
    public static double APRILTAG_TOLERANCE_MM = 300.0;  // Deprecated: no longer used in TeleOp

    // Simple Gain-based Control (instead of PIDF)
    // Drive = Error * Gain.  Make these values smaller for smoother control, or larger for more aggressive response.
    public static double SPEED_GAIN = 0.02;              // Forward/Backward speed control gain (0.02 = 50% power at 25 inch error)
    public static double TURN_GAIN = 0.01;               // Turning speed control gain (0.01 = 25% power at 25 degree error)
    public static double STRAFE_GAIN = 0.015;            // Strafing speed control gain for centering on tag

    // Maximum Auto-Drive Speeds (deprecated)
    @Deprecated
    public static double MAX_AUTO_SPEED = 0.5;
    @Deprecated
    public static double MAX_AUTO_TURN = 0.3;
    @Deprecated
    public static double MAX_AUTO_STRAFE = 0.4;

    // Camera Configuration - removed in favor of PedroPathing/Pinpoint
    @Deprecated
    public static boolean USE_WEBCAM = false;
    @Deprecated
    public static String WEBCAM_NAME = "Webcam 1";
    @Deprecated
    public static int CAMERA_EXPOSURE_MS = 6;
    @Deprecated
    public static int CAMERA_GAIN = 250;
    @Deprecated
    public static int APRILTAG_DECIMATION = 2;

    // Camera Mounting Configuration (kept for reference)
    @Deprecated
    public static boolean CAMERA_FACING_BACK = true;
    @Deprecated
    public static double CAMERA_OFFSET_X_CM = 14.5;
    @Deprecated
    public static double CAMERA_OFFSET_Y_CM = 0.0;
}
