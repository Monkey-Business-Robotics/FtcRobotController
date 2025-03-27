package org.firstinspires.ftc.tc25734;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.tc25734.MechanumDriveClass;
import org.firstinspires.ftc.tc25734.ClawSystemClass;
//import org.firstinspires.ftc.tc25734.ArmControl;

@TeleOp(name = "Mechanum Drive Woohoo!", group = "Linear Opmode")
public class TeleopMechanum extends LinearOpMode {
    private DcMotor slider;


    private static final int SLIDER_HIGH_BAR_POSITION = 720;
    private static final int SLIDER_MIN = 0; // 1000~=2in
    private static final int SLIDER_MAX = 19000;

    private DcMotor armMotorA;
    private DcMotor armMotorB;

    // PID coefficients
    private static final double kP = 0.12;  // Proportional gain
    private static final double kI = 0; // Integral gain
    private static final double kD = 0;  // Derivative gain

    private double integralSum = 0;
    private double previousError = 0;

    // Arm position constants
    private static final int ARM_POSITION_MIN = 0;
    private static final int ARM_POSITION_MID = 50;
    private static final int ARM_POSITION_MAX = 115;

    // varible used to determine if we want the PID to run
    boolean usePID = false;


    @Override
    public void runOpMode() {
        slider = hardwareMap.get(DcMotor.class, "slider");

        // Initialize the wrist motor
        armMotorA = hardwareMap.get(DcMotor.class, "wrist_a");
        armMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorB = hardwareMap.get(DcMotor.class, "wrist_b");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        int targetPositionWrist = 0;  // Start position wrist
        slider.setTargetPosition(0);
        slider.setPower(0);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setDirection(DcMotor.Direction.FORWARD);

        ClawSystemClass clawDriver = new ClawSystemClass(hardwareMap, telemetry);
        MechanumDriveClass mechanumDrive = new MechanumDriveClass(hardwareMap, telemetry);
        mechanumDrive.setMode(0);
        int sliderPosition = 0;
        Boolean clawOpen = false;
        //Boolean pivotDown = false;
        Boolean button_a_released = true;
        //Boolean button_b_released = true;
        //Boolean button_x_released = true
        ;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            mechanumDrive.runByPower(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            //clawDriver.moveWrist((gamepad2.right_stick_y / 1.7) * (0.62 * Math.pow(gamepad2.right_stick_y, 2) + 0.45));

            double power = -gamepad2.left_stick_y;
            sliderPosition = slider.getCurrentPosition();
            if ((sliderPosition <= SLIDER_MIN) && power < 0) {
                power = 0;
            }
            if ((sliderPosition >= SLIDER_MAX) && power > 0) {
                power = 0;
            }
            telemetry.addData("slider position", sliderPosition);
            slider.setPower(power);

            if (gamepad2.a && button_a_released) {
                clawOpen = !clawOpen;
                button_a_released = false;
            } else if (!gamepad2.a) {
                button_a_released = true;
            }
            clawDriver.setClawOpen(clawOpen);

            telemetry.addData("Status", "Running");
            telemetry.update();

            // Control the arm motor
            // Set target position based on gamepad buttons and reset encoders
            if (gamepad2.x) {
                armMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotorA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetPositionWrist = ARM_POSITION_MIN;
            }
            if (gamepad2.b) {
                targetPositionWrist = ARM_POSITION_MID;
                usePID = true;
            }
            if (gamepad2.y) {
                targetPositionWrist = ARM_POSITION_MAX;
                usePID = true;
            }
            if (gamepad2.right_stick_y >0){
                armMotorA.setPower(gamepad2.right_stick_y * .15 ) ;
                armMotorB.setPower(-(gamepad2.right_stick_y * .15));
                usePID = false;
            } else if (gamepad2.right_stick_y <0){
                armMotorA.setPower(gamepad2.right_stick_y *.37);
                armMotorB.setPower(-(gamepad2.right_stick_y *.37));
                usePID = false;

            } else if (usePID){
                // Apply PID control to move the arm to the target position
                double powerWrist = calculatePID(targetPositionWrist, armMotorA.getCurrentPosition());
                armMotorA.setPower(powerWrist);
                armMotorB.setPower(-powerWrist);
            }
            // Telemetry for debugging
            telemetry.addData("Target Position",targetPositionWrist);
            telemetry.addData("Current Position", armMotorA.getCurrentPosition());
            //telemetry.addData("Motor Power", power);
            telemetry.update();
        }
    }

    private double calculatePID(int targetPosition, int currentPosition) {
        // Calculate error
        double error = targetPosition - currentPosition;

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

        // Clamp output to valid motor power range [-1, 1]
        output = Math.max(-.75, Math.min(.6, output));

        // Update previous error for the next cycle
        previousError = error;

        return output;
    }
}


