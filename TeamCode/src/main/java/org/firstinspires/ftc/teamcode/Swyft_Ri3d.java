package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Import the DcMotorEx class
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="SWYFT_Ri3d")
//@Disabled
public class Swyft_Ri3d extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Drive Motors
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    // Mechanism Motors
    private DcMotor intake = null;

    private DcMotor upperintake = null;
    private DcMotorEx shooterL = null; // Using DcMotorEx to get velocity
    private DcMotor shooterR = null;
    private DcMotor climb = null;

    // Servos
    private Servo hold = null;

    //State Variables for Toggles
    boolean flywheelOn = false;
    boolean a_button_previously_pressed = false;

    // CONTROL CONSTANTS
    // Bang-Bang Controller Constants
    static final double BANG_BANG_TARGET_VELOCITY = 1500.0; // Target speed in ticks per second
    static final double FLYWHEEL_FULL_POWER = -1.0; // Power to apply when below target speed

    // Other Constants
    static final double INTAKE_POWER = 1.0;
    static final double CLIMB_POWER = 1.0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // --- HARDWARE MAPPING ---
        frontLeft  = hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "BackLeft");
        backRight  = hardwareMap.get(DcMotor.class, "BackRight");

        intake     = hardwareMap.get(DcMotor.class, "Intake");
        upperintake = hardwareMap.get(DcMotor.class, "UpperIntake");
        shooterL   = hardwareMap.get(DcMotorEx.class, "ShooterL"); // Mapped as DcMotorEx to read encoder
        shooterR   = hardwareMap.get(DcMotor.class, "ShooterR");
        climb      = hardwareMap.get(DcMotor.class, "Climb");
        hold       = hardwareMap.get(Servo.class, "hold");

        // MOTOR DIRECTION
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        shooterR.setDirection(DcMotor.Direction.REVERSE);
        shooterL.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        climb.setDirection(DcMotor.Direction.FORWARD);

        // MOTOR BEHAVIOR
        // Drivetrain and Climber set to BRAKE
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        climb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Intake and Shooters set to FLOAT (Coast)
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // ENCODER SETUP FOR SHOOTER
        shooterL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set other motors to run without encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        climb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hold.setPosition(0.5);

        telemetry.addData("Status", "Initialized. Ready to run!");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            // STANDARD FTC DRIVE SCHEME
            double y  = -gamepad1.left_stick_y;   // Forward/back
            double x  =  gamepad1.left_stick_x;   // Strafe
            double rx =  gamepad1.right_stick_x;  // Rotation


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            //Intake balls
            if (gamepad1.x) {
                intake.setPower(1);
                upperintake.setPower(1);
                hold.setPosition(1);
            }

            //Outtake Balls
            if (gamepad1.y) {
                intake.setPower(-1);
                upperintake.setPower(-1);
            }

            if (gamepad1.a && !a_button_previously_pressed) {                  
                flywheelOn = !flywheelOn;
            }
            a_button_previously_pressed = gamepad1.a;

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
                    //Run intake to shoot balls
                    intake.setPower(1);
                    upperintake.setPower(1);
                }
            } else {
                shooterL.setPower(0);
                shooterR.setPower(0);
                //Lock shooting
                hold.setPosition(1);
            }


            if (gamepad1.dpad_up) {
                climb.setPower(CLIMB_POWER);
            } else if (gamepad1.dpad_down) {
                climb.setPower(-CLIMB_POWER);
            } else {
                climb.setPower(0);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Flywheel Status", flywheelOn ? "ON" : "OFF");
            telemetry.addData("Target Velocity", BANG_BANG_TARGET_VELOCITY);
            telemetry.addData("Actual Velocity", "%.2f", shooterL.getVelocity());
            telemetry.addData("Shooter Power", "%.2f", shooterL.getPower());
            telemetry.update();
        }
    }
}