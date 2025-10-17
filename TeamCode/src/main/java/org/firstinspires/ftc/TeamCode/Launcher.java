package org.firstinspires.ftc.TeamCode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Launcher {
    DcMotor launcher_a;
    DcMotor launcher_b;
    Servo loading_servo;
    Servo launch_servo;

    Telemetry telemetry;

    public static double LAUNCH_SPEED_FAR = 0.83;
    public static double LAUNCH_SPEED_CLOSE = 0.40;

    public static double LAUNCH_ANGLE_CLOSE = 0;
    public static double LAUNCH_ANGLE_FAR = 0;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        launcher_a = hardwareMap.get(DcMotor.class, "launch_a");
        launcher_b = hardwareMap.get(DcMotor.class, "launch_b");

        loading_servo = hardwareMap.get(Servo.class, "load_servo");
        launch_servo = hardwareMap.get(Servo.class, "launch_servo");

        loading_servo.setPosition(1);

        this.telemetry = telemetry;
    }

    public void stop() {
        launcher_a.setPower(0);
        launcher_b.setPower(0);
    }

    public void readyLaunchClose() {
        launcher_a.setPower(LAUNCH_SPEED_CLOSE);
        launcher_b.setPower(-LAUNCH_SPEED_CLOSE);
        launch_servo.setPosition(LAUNCH_ANGLE_CLOSE);
    }

    public void readyLaunchFar() {
        launcher_a.setPower(LAUNCH_SPEED_FAR);
        launcher_b.setPower(-LAUNCH_SPEED_FAR);
        launch_servo.setPosition(LAUNCH_ANGLE_FAR);
    }

    public void launchIfReady() {
        // move loading servo to launch position then back
        loading_servo.setPosition(0.5);
    }

    public void reset() {
        loading_servo.setPosition(120);
    }
}
