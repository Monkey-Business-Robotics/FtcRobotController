package org.firstinspires.ftc.TeamCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Configurable
@TeleOp(name = "Standard Teleop", group = "Linear Opmode")
public class MecanumTeleop extends LinearOpMode {

    public static PIDCoefficients targetLockPIDCoeff = new PIDCoefficients(0.028, 0.00075, 0.02);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        /*
         * Set up the mecanum driver for analog control
         */
        MecanumDriver mecanumDriver = new MecanumDriver(hardwareMap, telemetry, this);
        mecanumDriver.setMode(0);

        /*
         * Configure the limelight & enable pipeline 0
         */
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        /*
         * Set up the target lock on PID loop
         */
        PIDControl targetLockPID = new PIDControl(targetLockPIDCoeff, 0.1);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get the latest limelight result for use in target lock
            LLResult result = limelight.getLatestResult();

            /*
             * If Gamepad button A is pressed, lock onto the target april tag.
             *
             * Note: we may have to change to tracking only specific april tags in the pipeline,
             *       but we should try this first.
             */
//            double powerScale = gamepad1.right_bumper ? 1.0  : (gamepad1.left_bumper ? 0.5 : 1); //Check if the right bumper is held
            if (gamepad1.a) {
                double rotation = targetLockPID.update(0, result.getTx());
                mecanumDriver.runByPower(gamepad1.left_stick_x, gamepad1.left_stick_y, -rotation, 1);
            } else {
                targetLockPID.reset();
                mecanumDriver.runByPower(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, 1);
            }

            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}


