package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class AxonSubsystem {
    private Servo intakeAxonLeft, intakeAxonRight, outtakeAxonLeft, outtakeAxonRight;

    public AxonSubsystem(HardwareMap hardwareMap) {
        intakeAxonLeft = hardwareMap.get(Servo.class, "intakeAxonLeft");
        intakeAxonRight = hardwareMap.get(Servo.class, "intakeAxonRight");
        outtakeAxonLeft = hardwareMap.get(Servo.class, "outtakeAxonLeft");
        outtakeAxonRight = hardwareMap.get(Servo.class, "outtakeAxonRight");
    }

    //------------------------------IntakeAxon------------------------------//

    public void SetIntakeAxonPosition(double position) {
        intakeAxonLeft.setPosition(position);
        intakeAxonRight.setPosition(position);
    }

    //------------------------------OuttakeAxon------------------------------//

    public void SetOuttakeAxonPosition(double position) {
        outtakeAxonLeft.setPosition(position);
        outtakeAxonRight.setPosition(position);
    }

    //------------------------------Getters------------------------------//

    public double getAxonIntakePosition() {
        return intakeAxonLeft.getPosition();
    }

    public double getAxonOuttakePosition() {
        return outtakeAxonLeft.getPosition();
    }
}
