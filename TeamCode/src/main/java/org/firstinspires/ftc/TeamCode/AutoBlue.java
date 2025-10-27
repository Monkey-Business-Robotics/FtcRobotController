package org.firstinspires.ftc.TeamCode;

import static java.lang.Math.abs;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;


@Configurable
@Autonomous(name = "Blue", group = "Linear Opmode")
public class AutoBlue extends LinearOpMode {

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
        mecanumDriver.setMode(1);

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

        mecanumDriver.moveX(35, 0.5);

        mecanumDriver.setMode(0);
        LLResult result = limelight.getLatestResult();
        while(abs(result.getTx()) > 1) {
            result = limelight.getLatestResult();
            double pid = targetLockPID.update(0, result.getTx());
            mecanumDriver.runByPower(0, 0, -pid, 1);
        }
        mecanumDriver.runByPower(0, 0, 0, 0);
        targetLockPID.reset();
        mecanumDriver.setMode(1);

        launcher.readyLaunchFar();
        sleep(1000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.reset();
        sleep(1000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.reset();
        sleep(1000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.reset();
        sleep(1000);
        launcher.launchIfReady();
        sleep(1000);
        launcher.stop();
        launcher.reset();

        mecanumDriver.moveX(250, 0.5);
    }
}
