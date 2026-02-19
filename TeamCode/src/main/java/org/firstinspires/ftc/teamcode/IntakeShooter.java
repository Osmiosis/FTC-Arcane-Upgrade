package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeShooter {

    // Mechanism Motors
    private DcMotor intake = null;
    private DcMotor upperintake = null;
    private DcMotorEx shooterL = null; // Using DcMotorEx to get velocity
    private DcMotor shooterR = null;

    // Servos
    private Servo hold = null;

    // State Variables for Toggles
    private boolean flywheelOn = false;
    private boolean a_button_previously_pressed = false;

    // CONTROL CONSTANTS
    // Bang-Bang Controller Constants
    public static final double BANG_BANG_TARGET_VELOCITY = 1500.0; // Target speed in ticks per second
    public static final double FLYWHEEL_FULL_POWER = -1.0; // Power to apply when below target speed

    // Other Constants
    public static final double INTAKE_POWER = 1.0;

    public IntakeShooter(HardwareMap hardwareMap) {
        // Hardware mapping
        intake = hardwareMap.get(DcMotor.class, "Intake");
        upperintake = hardwareMap.get(DcMotor.class, "UpperIntake");
        shooterL = hardwareMap.get(DcMotorEx.class, "ShooterL"); // Mapped as DcMotorEx to read encoder
        shooterR = hardwareMap.get(DcMotor.class, "ShooterR");
        hold = hardwareMap.get(Servo.class, "hold");

        // Motor Direction
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterL.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Motor Behavior - Intake and Shooters set to FLOAT (Coast)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Encoder setup for shooter
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set other motors to run without encoders
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servo position
        hold.setPosition(0.5);
    }

    public void update(Gamepad gamepad) {
        // Intake balls (X button)
        if (gamepad.x) {
            intake.setPower(1);
            upperintake.setPower(1);
            hold.setPosition(1);
        }

        // Outtake balls (Y button)
        if (gamepad.y) {
            intake.setPower(-1);
            upperintake.setPower(-1);
        }

        // Toggle flywheel on/off (A button)
        if (gamepad.a && !a_button_previously_pressed) {
            flywheelOn = !flywheelOn;
        }
        a_button_previously_pressed = gamepad.a;

        if (flywheelOn) {
            // Get the current velocity from the encoded motor
            double currentVelocity = -shooterL.getVelocity();

            // Bang-Bang Control Logic
            if (currentVelocity < BANG_BANG_TARGET_VELOCITY) {
                // If speed is too low, turn motors to full power
                shooterL.setPower(FLYWHEEL_FULL_POWER);
                shooterR.setPower(FLYWHEEL_FULL_POWER);
                hold.setPosition(1);
            } else {
                // If speed is at or above target, turn motors off (coast)
                shooterL.setPower(0);
                shooterR.setPower(0);
                hold.setPosition(0);
                // Run intake to shoot balls
                intake.setPower(1);
                upperintake.setPower(1);
            }
        } else {
            shooterL.setPower(0);
            shooterR.setPower(0);
            // Lock shooting
            hold.setPosition(1);
        }
    }

    public void stop() {
        intake.setPower(0);
        upperintake.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
        hold.setPosition(0.5);
        flywheelOn = false;
    }

    // Getter methods for telemetry
    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public double getShooterVelocity() {
        return shooterL.getVelocity();
    }

    public double getShooterPower() {
        return shooterL.getPower();
    }

    public double getTargetVelocity() {
        return BANG_BANG_TARGET_VELOCITY;
    }
}

