package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Basic: Omni Linear OpMode with Intake + Launcher (Bumpers) + Servo", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor launcherMotor = null;  // Launcher motor (Expansion Hub port 1)
    private Servo launcherServo = null;

    // Servo positions
    private final double SERVO_REST = 0.0;   // resting position
    private final double SERVO_FIRE = 0.5;   // ~90 degrees

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backRightDrive");
        intakeMotor     = hardwareMap.get(DcMotor.class, "intakeMotor");
        launcherMotor   = hardwareMap.get(DcMotor.class, "launcherMotor");
        launcherServo   = hardwareMap.get(Servo.class, "launcherServo");
        // Set motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor.setDirection(DcMotor.Direction.FORWARD);

        // Initialize servo position
        launcherServo.setPosition(SERVO_REST);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double axial;   // Forward/back
            double lateral; // Left/right
            double yaw;     // Rotation

            // Joystick mapping
            axial   = -gamepad1.left_stick_y;
            lateral =  gamepad1.left_stick_x;
            yaw     =  gamepad1.right_stick_x;

            // Calculate wheel powers
            double frontLeftPower  = axial - lateral + yaw;
            double frontRightPower = axial + lateral - yaw;
            double backLeftPower   = axial + lateral + yaw;
            double backRightPower  = axial - lateral - yaw;

            // Normalize powers
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            // Drive
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // --- Intake control (A/B) ---
            if (gamepad1.a) {
                intakeMotor.setPower(1.0);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            // --- Launcher motor (RB/LB) ---
            if (gamepad1.right_bumper) {
                launcherMotor.setPower(0.8); // forward spin
            } else if (gamepad1.left_bumper) {
                launcherMotor.setPower(-0.8); // reverse spin
            } else {
                launcherMotor.setPower(0.0); // stop
            }

            // --- Launcher Servo (X/Y) ---
            if (gamepad1.x) {
                launcherServo.setPosition(SERVO_FIRE); // Rotate 90 degrees
            } else if (gamepad1.y) {
                launcherServo.setPosition(SERVO_REST); // Return to rest
            }

            // Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Drive Power", "FL(%.2f) FR(%.2f) BL(%.2f) BR(%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.addData("Intake", "Power: %.2f", intakeMotor.getPower());
            telemetry.addData("Launcher", "Power: %.2f", launcherMotor.getPower());
            telemetry.addData("Servo", "Position: %.2f", launcherServo.getPosition());
            telemetry.update();
        }
    }
}
