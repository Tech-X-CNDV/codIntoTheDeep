package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ClawSubsystem {
    private Servo pivotIntake, grabIntake, grabOuttake;

    public ClawSubsystem(HardwareMap hardwareMap) {
        pivotIntake = hardwareMap.get(Servo.class, "rotireGhiaraIntake");
        grabIntake = hardwareMap.get(Servo.class, "ghiaraIntake");
        grabOuttake = hardwareMap.get(Servo.class, "ghiaraOuttake");
    }

    //------------------------------Grab------------------------------//

    public void closeIntakeClaw() {
        grabIntake.setPosition(RobotConstants.closePos);
    }

    public void openIntakeClaw() {
        grabIntake.setPosition(RobotConstants.intakeOpenPos);
    }

    public void closeOuttakeClaw() {
        grabIntake.setPosition(RobotConstants.closePos);
    }

    public void openOuttakeClaw() {
        grabIntake.setPosition(RobotConstants.outtakeOpenPos);
    }

    //------------------------------Pivot------------------------------//

    public void initialRotClaw() {
        pivotIntake.setPosition(RobotConstants.minPosRotInt);
    }

    public void rotatedClaw() {
        pivotIntake.setPosition(RobotConstants.maxPosRotInt);
    }

    //------------------------------Getters------------------------------//

    public double getGrabIntakePosition() {
        return grabIntake.getPosition();
    }

    public double getPivotIntakePosition() {
        return pivotIntake.getPosition();
    }

    public double getGrabOuttakePosition() {
        return grabOuttake.getPosition();
    }
}
