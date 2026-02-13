package org.firstinspires.ftc.teamcode;

// Import classes for LinearOpMode, TeleOp annotation, and motor control
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //===========  Initialize hardware ===========//
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intakeMotor");
        DcMotor launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        DcMotor launcherMotor2 = hardwareMap.dcMotor.get("launcherMotor2");

        Servo launcherServo = hardwareMap.servo.get("launcherServo");
        launcherServo.setPosition(0);

        ElapsedTime servoTimer = new ElapsedTime();
        boolean servoActive = false;
        double SERVO_UP = 0.8;     // forward position (tune this)
        double SERVO_DOWN = 0.0;   // resting position (tune this)
        double SERVO_UP_TIME = 0.12;
        boolean bBumperState = false;
        boolean leftBumperState = false;
        boolean rightBumperState = false;
        boolean launchForward = false;
        boolean intakeForward = false;
        boolean intakeBackwards = false;

        // Launch power constant
        double launcherPower = 0.55;

        // D-pad state tracking
        boolean dpadUpPrev = false;
        boolean dpadDownPrev = false;
        boolean dpadLeftPrev = false;
        boolean dpadRightPrev = false;

        // motor directions (per-motor, verified)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);


        // ensure motors stop immediately when power is zero
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherServo.setPosition(SERVO_DOWN);

        waitForStart();
        if (isStopRequested()) return;

        // ==========================
        // Main TeleOp Loop
        // ==========================
        while (opModeIsActive()) {

            // a) Read joystick inputs
            double y  = -gamepad1.left_stick_y;
            double x  =  gamepad1.left_stick_x;
            double rx =  gamepad1.right_stick_x;

            // b) Calculate denominator
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            // c) Mecanum math
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x - rx) / denominator;

            // d) Apply drive power
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // e) Adjust flywheel speed
            if (gamepad1.dpad_up && !dpadUpPrev) {
                launcherPower += 0.125;
            } else if (gamepad1.dpad_down && !dpadDownPrev) {
                launcherPower -= 0.125;
            } else if (gamepad1.dpad_left && !dpadLeftPrev) {
                launcherPower = 0.75;
            } else if (gamepad1.dpad_right && !dpadRightPrev) {
                launcherPower = 0.55;
            }

            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;
            dpadLeftPrev = gamepad1.dpad_left;
            dpadRightPrev = gamepad1.dpad_right;


            // f) Launcher motor
            if (gamepad1.b && !bBumperState) {
                launchForward = !launchForward;
            }

            bBumperState = gamepad1.b;

            if (launchForward) {
                launcherMotor.setPower(launcherPower);
                launcherMotor2.setPower(launcherPower);
            } else {
                launcherMotor.setPower(0.0);
                launcherMotor2.setPower(0.0);
            }

            // ======= ADDED TELEMETRY (ONLY ADDITION) =======
            telemetry.addData("Launcher Power (set)", "%.2f", launcherPower);
            telemetry.addData("Launcher Motor Output", "%.2f", launcherMotor.getPower());
            telemetry.update();
            // ==============================================

            if (gamepad1.left_bumper && !leftBumperState) {
                if (intakeForward) {
                    intakeForward = false;
                } else {
                    intakeForward = true;
                    intakeBackwards = false;
                }

            }

            if (gamepad1.right_bumper && !rightBumperState) {
                if(intakeBackwards) {
                    intakeBackwards = false;
                } else {
                    intakeBackwards = true;
                    intakeForward = false;
                }
            }

            leftBumperState = gamepad1.left_bumper;
            rightBumperState = gamepad1.right_bumper;

            if (intakeForward) {
                intakeMotor.setPower(1.0);
            } else if (intakeBackwards){
                intakeMotor.setPower(-1.0);
            } else {
                intakeMotor.setPower(0.0);
            }

            if (gamepad1.a && !servoActive) {
                launcherServo.setPosition(SERVO_UP);
                servoTimer.reset();
                servoActive = true;
            }

            if (servoActive && servoTimer.seconds() >= SERVO_UP_TIME) {
                launcherServo.setPosition(SERVO_DOWN);
                servoActive = false;
            }
        }
    }
}
