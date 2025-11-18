package org.firstinspires.ftc.TeamCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Launcher {
    DcMotorEx launcher_a;
    DcMotorEx launcher_b;
    DcMotor intake;
    //Servo loading_servo;
    Servo launch_servo;


    Telemetry telemetry;

    public static double LAUNCH_SPEED_FAR = 0.7; // / 60 = RPM -> RPS
    public static double LAUNCH_SPEED_CLOSE = 0.55;
    public static double DISTANCE_FAR = 3.5;
    public static double DISTANCE_CLOSE = 1.2;

    public static double LAUNCH_RAMP_OPEN = 0;
    public static double INTAKE_SPEED = 0.8;
    public static int TICKS_TO_RUN_BACK = 25;

    private static int ticks_since_ready_launch = 0;


    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        launcher_a = hardwareMap.get(DcMotorEx.class, "launch_a");
        launcher_b = hardwareMap.get(DcMotorEx.class, "launch_b");
        intake = hardwareMap.get(DcMotor.class, "intake");


       // loading_servo = hardwareMap.get(Servo.class, "load_servo");
        launch_servo = hardwareMap.get(Servo.class, "launch_servo");

       // loading_servo.setPosition(1);
        launch_servo.setPosition(0.0);

        this.telemetry = telemetry;
    }

    public void stop() {
        launcher_a.setPower(0);
        launcher_b.setPower(0);
        ticks_since_ready_launch = 0;
    }

    public void readyLaunch(double distance) {
        double launch_speed = ((LAUNCH_SPEED_FAR - LAUNCH_SPEED_CLOSE) / (DISTANCE_FAR - DISTANCE_CLOSE)) * (distance - DISTANCE_FAR) + LAUNCH_SPEED_FAR;
        telemetry.addData("power", launch_speed);
        if (ticks_since_ready_launch < TICKS_TO_RUN_BACK) {
            launcher_a.setPower(-launch_speed);
            launcher_b.setPower(launch_speed);
        } else {
            launcher_a.setPower(launch_speed);
            launcher_b.setPower(-launch_speed);
        }
        ticks_since_ready_launch ++;
    }

    public void readyLaunchAuto(double distance) {
        double launch_speed = ((LAUNCH_SPEED_FAR - LAUNCH_SPEED_CLOSE) / (DISTANCE_FAR - DISTANCE_CLOSE)) * (distance - DISTANCE_FAR) + LAUNCH_SPEED_FAR;
        telemetry.addData("power", launch_speed);
        launcher_a.setPower(launch_speed);
        launcher_b.setPower(-launch_speed);
        ticks_since_ready_launch ++;
    }

    public void readyIntake(){
        intake.setPower(INTAKE_SPEED);
    }

    public void launchIfReady() {
        // move loading servo to launch position then back
        launch_servo.setPosition(0.3);
    }

    public void reset() {
        launch_servo.setPosition(0.1);
        telemetry.addData("launch servo pos", launch_servo.getPosition());
    }

    public void StopIntake() {
        intake.setPower(0);
    }

    public double speed() {
        return (-launcher_b.getVelocity() * (60.0 / 28.0));
    }

    public boolean isReady(double dist) {
        double REQUIRED_SPEED_FAR = 3300;
        double REQUIRED_SPEED_SHORT = 2500;
        double launch_speed = ((REQUIRED_SPEED_FAR - REQUIRED_SPEED_SHORT) / (DISTANCE_FAR - DISTANCE_CLOSE)) * (dist - DISTANCE_FAR) + REQUIRED_SPEED_FAR;
        return speed() > launch_speed;
    }

}
