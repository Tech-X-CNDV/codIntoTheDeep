package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class AxonSubsystem {
    private final Servo intakeAxonLeft, intakeAxonRight, outtakeAxonLeft, outtakeAxonRight;

    public AxonSubsystem(HardwareMap hardwareMap) {
        intakeAxonLeft = hardwareMap.get(Servo.class, "intakeAxonLeft");
        intakeAxonRight = hardwareMap.get(Servo.class, "intakeAxonRight");
        outtakeAxonLeft = hardwareMap.get(Servo.class, "outtakeAxonLeft");
        outtakeAxonRight = hardwareMap.get(Servo.class, "outtakeAxonRight");
    }

    //------------------------------IntakeAxon------------------------------//

    public void InitIntake(){
        intakeAxonLeft.setPosition(RobotConstants.intakeUpPos);
        intakeAxonRight.setPosition(RobotConstants.intakeUpPos);
    }

    public void SetIntakePosition(double position) {
        intakeAxonLeft.setPosition(position);
        intakeAxonRight.setPosition(position);
    }

    //------------------------------OuttakeAxon------------------------------//

    public void InitOuttake(double position){
        outtakeAxonLeft.setPosition(position);
        outtakeAxonRight.setPosition(position);
    }

    public void SetOuttakePosition(double position) {
        outtakeAxonLeft.setPosition(position);
        outtakeAxonRight.setPosition(position);
    }

    //------------------------------Getters------------------------------//

    public double GetIntakePosition() {
        return intakeAxonLeft.getPosition();
    }

    public double GetOuttakePosition() {
        return outtakeAxonLeft.getPosition();
    }
}
