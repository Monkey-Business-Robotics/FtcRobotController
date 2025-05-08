package org.firstinspires.ftc.tc25734;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Standard Teleop", group = "Linear Opmode")
public class MecanumTeleop extends LinearOpMode {

    private DcMotor armMotor;

    // PID coefficients
    private static final double kP = .0005;  // Proportional gain
    private static final double kI = 0; // Integral gain
    private static final double kD = 0;  // Derivative gain
    private static final double tolerance = 1;

    // PID state
    private double integralSum = 0;
    private double previousError = 0;
    double error = 0;

    // Arm position constants
    private static final int ARM_POSITION_MID = 1350;
    private static final int ARM_POSITION_HOLD = 2700;
    private static final int ARM_POSITION_PICKUP = 3025;
    private static final int ARM_POSITION_MAX = 3400;

    @Override
    public void runOpMode() {
//        DcMotor slider = hardwareMap.get(DcMotor.class, "slider");

        // Initialize the wrist motor
//        armMotor = hardwareMap.get(DcMotor.class, "wrist_a");
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize the autonomous bar touching servo motor
//        Servo miniArm = hardwareMap.get(Servo.class, "mini_arm");
//        miniArm.setDirection(Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

//        miniArm.setPosition(0);

        double targetPositionWrist = 0;  // Start position wrist
//        slider.setTargetPosition(0);
//        slider.setPower(0);
//        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slider.setDirection(DcMotor.Direction.FORWARD);

//        ClawSystemClass clawDriver = new ClawSystemClass(hardwareMap, telemetry);
        MecanumDriver mecanumDriver = new MecanumDriver(hardwareMap, telemetry, this);
        mecanumDriver.setMode(0);
        int sliderPosition;
        boolean clawOpen = false;
        boolean button_a_released = true;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double powerScale = gamepad1.right_bumper ? 1.0  : (gamepad1.left_bumper ? 0.5 : .75); //Check if the right bumper is held
            mecanumDriver.runByPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, powerScale);

            double power = -gamepad2.left_stick_y;
//            sliderPosition = slider.getCurrentPosition();
//            if ((sliderPosition <= SLIDER_MIN) && power < 0) {
//                power = 0;
//            }
//            if ((sliderPosition >= SLIDER_MAX) && power > 0) {
//                power = 0;
//            }
//            telemetry.addData("slider position", sliderPosition);
//            slider.setPower(power);

            if (gamepad2.a && button_a_released) {
                clawOpen = !clawOpen;
                button_a_released = false;
            } else if (!gamepad2.a) {
                button_a_released = true;
            }
//            clawDriver.setClawOpen(clawOpen);

            telemetry.addData("Status", "Running");
            telemetry.update();

            // Control the arm motor
            // Set target position based on gamepad buttons and reset encoders
//            if (gamepad2.b) {
//                targetPositionWrist = ARM_POSITION_MID;
//            }
//            if (gamepad2.y) {
//                targetPositionWrist = ARM_POSITION_HOLD;
//            }
//            if (gamepad2.right_bumper){
//                targetPositionWrist = ARM_POSITION_PICKUP;
//            }
//            if (gamepad2.right_stick_y >0){
//                targetPositionWrist += gamepad2.right_stick_y * 10;
//            } else if (gamepad2.right_stick_y <0){
//                targetPositionWrist += gamepad2.right_stick_y * 10;
//            }

            // Limit the range of motion
//            if (targetPositionWrist < 0) {
//                targetPositionWrist = 0;
//            }
//            if (targetPositionWrist > ARM_POSITION_MAX) {
//                targetPositionWrist = ARM_POSITION_MAX;
//            }

            // Apply PID control to move the arm to the target position
//            double powerWrist = calculatePID((int) targetPositionWrist, -armMotor.getCurrentPosition());
//            armMotor.setPower(-powerWrist);

            // Telemetry for debugging
            telemetry.addData("Target Position",targetPositionWrist);
//            telemetry.addData("Current Position", -armMotor.getCurrentPosition());
            //telemetry.addData("Motor Power", power);
        }
    }

    private double calculatePID(int targetPosition, int currentPosition) {
        // Calculate error
        error = targetPosition - Math.abs(armMotor.getCurrentPosition());

        // Proportional term
        double proportional = kP * error;

        // Integral term
        integralSum += error;
        double integral = kI * integralSum;

        // Derivative term
        double derivative = error - previousError;
        double derivativeTerm = kD * derivative;

        // Combine PID terms
        double output = proportional + integral + derivativeTerm;

        double distanceToTarget = Math.abs(error);
        if (distanceToTarget < 2 * tolerance) {
            output *=distanceToTarget / 10 * tolerance;
        }

        // Clamp output to valid motor power range [-1, 1]
        output = Math.max(-.5, Math.min(.5, output));

        // Update previous error for the next cycle
        previousError = error;

        // Stop the motor if within tolerance
        if (Math.abs(error) <= tolerance) {
            output = (0);
        }

        return output;
    }
}


