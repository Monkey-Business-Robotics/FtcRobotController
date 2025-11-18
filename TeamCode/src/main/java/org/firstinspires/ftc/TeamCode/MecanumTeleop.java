package org.firstinspires.ftc.TeamCode;

import static java.lang.Math.pow;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.ArrayList;
import java.util.Arrays;

@Configurable
@TeleOp(name = "Standard Teleop", group = "Linear Opmode")
public class MecanumTeleop extends LinearOpMode {

    public static PIDCoefficients targetLockPIDCoeff = new PIDCoefficients(0.028, 0.00074, 0.035);

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
         * Configure the launcher
         */
        Launcher launcher = new Launcher(hardwareMap, telemetry);

        /*
         * Set up the target lock on PID loop
         */
        PIDControl targetLockPID = new PIDControl(targetLockPIDCoeff, 0.1);

        float speed_mult = 0.75f;
        boolean rumbled_locked = false;
        boolean rumbled_unlocked = false;

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
                rumbled_unlocked = false;
                if (!rumbled_locked) {
                    gamepad1.rumble(500);
                    rumbled_locked = true;
                }
                double rotation = targetLockPID.update(0, result.getTx());
                mecanumDriver.runByPower(
                        gamepad1.left_stick_x * speed_mult,
                        -gamepad1.left_stick_y * speed_mult,
                        -rotation, 1);
            } else {
                rumbled_locked = false;
                if (!rumbled_unlocked) {
                    gamepad1.rumble(1000);
                    rumbled_unlocked = true;
                }
                targetLockPID.reset();
                mecanumDriver.runByPower(
                        gamepad1.left_stick_x,
                        -gamepad1.left_stick_y,
                        -gamepad1.right_stick_x, speed_mult);
            }

            double ta = result.getTa();
            double dist = pow(2.95 / ta, 0.5);
            telemetry.addData("Distance", dist);
            if (gamepad1.left_bumper) {
                launcher.readyLaunch(dist);
            } else {
                launcher.stop();
            }

            if (gamepad1.right_bumper) {
                launcher.launchIfReady();
            } else {
                launcher.reset();
            }

            if (gamepad1.x) {
                launcher.readyIntake();
            } else {
                launcher.StopIntake();

            }

            if (gamepad1.left_trigger > 0.1) {
                speed_mult = 1;
            } else if (gamepad1.right_trigger > 0.1) {
                speed_mult = 0.25f;
            } else {
                speed_mult = 0.75f;
            }

            if(launcher.isReady(dist)) {
                gamepad2.setLedColor(0,1,0,100);
            } else {
                gamepad2.setLedColor(1,0,0,100);
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Speed", launcher.speed());
            telemetry.update();
        }
    }
}


