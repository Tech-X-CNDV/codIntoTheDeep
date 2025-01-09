package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ClawSubsystem {
    private final Servo pivotIntake, grabIntake, grabOuttake;

    public ClawSubsystem(HardwareMap hardwareMap) {
        pivotIntake = hardwareMap.get(Servo.class, "rotireGhiaraIntake");
        grabIntake = hardwareMap.get(Servo.class, "ghiaraIntake");
        grabOuttake = hardwareMap.get(Servo.class, "ghiaraOuttake");
    }

    //------------------------------Grab------------------------------//

    public void InitIntake(){
        grabIntake.setPosition(RobotConstants.closePos);
    }

    public void CloseIntake() {
        grabIntake.setPosition(RobotConstants.closePos);
    }

    public void OpenIntake() {
        grabIntake.setPosition(RobotConstants.intakeOpenPos);
    }

    public void InitOuttake(){
        grabOuttake.setPosition(RobotConstants.closePos);
    }

    public void CloseOuttake() {
        grabOuttake.setPosition(RobotConstants.closePos);
    }

    public void OpenOuttake() {
        grabOuttake.setPosition(RobotConstants.outtakeOpenPos);
    }

    //------------------------------Pivot------------------------------//

    public void InitPivot(){
        pivotIntake.setPosition(RobotConstants.minPosRotInt);
    }

    public void InitialRot() {
        pivotIntake.setPosition(RobotConstants.minPosRotInt);
    }

    public void Rotated() {
        pivotIntake.setPosition(RobotConstants.maxPosRotInt);
    }

    //------------------------------Getters------------------------------//

    public double GetGrabIntakePosition() {
        return grabIntake.getPosition();
    }

    public double GetPivotIntakePosition() {
        return pivotIntake.getPosition();
    }

    public double GetGrabOuttakePosition() {
        return grabOuttake.getPosition();
    }
}
