package org.firstinspires.ftc.tc25734;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ClawSystemClass {

    private final Telemetry telemetry;

    private final Servo clawServoA;
    private final Servo clawServoB;
    
    private final static double CLAW_CLOSED_POSITION_A = 0.5;
    private final static double CLAW_OPEN_POSITION_A = 1.0;
    private final static double CLAW_CLOSED_POSITION_B = 0.8;
    private final static double CLAW_OPEN_POSITION_B = 0.5;

    public ClawSystemClass(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        clawServoA = hardwareMap.get(Servo.class, "claw_a");
        clawServoB = hardwareMap.get(Servo.class, "claw_b");
    }
    
    public void setClawOpen(Boolean open) {
        double clawPositionA, clawPositionB;
        if (open) {
            clawPositionA = CLAW_OPEN_POSITION_A;
            clawPositionB = CLAW_OPEN_POSITION_B;
        } else {
            clawPositionA = CLAW_CLOSED_POSITION_A;
            clawPositionB = CLAW_CLOSED_POSITION_B;
        }
        
        telemetry.addData("Claw A:", clawPositionA);
        telemetry.addData("Claw B:", clawPositionB);
        clawServoA.setPosition(clawPositionA);
        clawServoB.setPosition(clawPositionB);
    }
 }   

    

