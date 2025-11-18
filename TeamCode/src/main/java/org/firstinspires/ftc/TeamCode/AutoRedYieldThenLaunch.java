package org.firstinspires.ftc.TeamCode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


@Configurable
@Autonomous(name = "Red Yield Then Launch", group = "Linear Opmode")
public class AutoRedYieldThenLaunch extends LinearOpMode {

    public static PIDCoefficients targetLockPIDCoeff = new PIDCoefficients(0.020, 0.00074, 0.035);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        /*
         * Set up the mecanum driver for analog control
         */
        MecanumDriver mecanumDriver = new MecanumDriver(hardwareMap, telemetry, this);
        mecanumDriver.setMode(1);

        /*
         * Configure the limelight & enable pipeline 0
         */
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(3);
        limelight.start();

        /*
         * Configure the launcher
         */
        Launcher launcher = new Launcher(hardwareMap, telemetry);

        /*
         * Set up the target lock on PID loop
         */
        PIDControl targetLockPID = new PIDControl(targetLockPIDCoeff, 0.1);

//        mecanumDriver.moveY(35, 0.5);
        mecanumDriver.moveX(250, 0.5);
        sleep(8000);
        mecanumDriver.moveX(-300, 0.5);

        mecanumDriver.setMode(0);
        LLResult result = limelight.getLatestResult();
        double pid = 10, filtered_tx = result.getTx();
        while((abs(pid) > 0.1 || abs(filtered_tx) > 0.05) && opModeIsActive()) {
            result = limelight.getLatestResult();
            filtered_tx += result.getTx();
            filtered_tx /= 2;
            pid = targetLockPID.update(0, result.getTx());
            mecanumDriver.runByPower(0, 0, -pid, 1);
        }
        mecanumDriver.runByPower(0, 0, 0, 0);
        targetLockPID.reset();
        mecanumDriver.setMode(1);

        launcher.readyLaunch(3.5);
        sleep(4000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.reset();
        sleep(2000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.reset();
        sleep(2000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.stop();
        launcher.reset();

        mecanumDriver.moveX(300, 0.5);
        sleep(2000);
    }
}
