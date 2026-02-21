package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class MecanumDrive {

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
        // Name strings must match up with the config on the Robot Controller app.
        front_left   = hardwareMap.get(DcMotor.class, "lf");
        front_right  = hardwareMap.get(DcMotor.class, "rf");
        back_left    = hardwareMap.get(DcMotor.class, "lr");
        back_right   = hardwareMap.get(DcMotor.class, "rr");

        // Set all motors to brake mode for precise stopping
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
        double drive  = gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double twist  = gamepad.right_stick_x;

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