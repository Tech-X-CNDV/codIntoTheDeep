package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class SliderSubsystem {
    private DcMotor outtakeSliderUp, outtakeSliderDown, intakeSlider;

    public SliderSubsystem(HardwareMap hardwareMap) {
        outtakeSliderUp = hardwareMap.get(DcMotor.class, "outtakeSliderUp");
        outtakeSliderUp.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSliderDown = hardwareMap.get(DcMotor.class, "outtakeSliderDown");
        outtakeSliderDown.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlider = hardwareMap.get(DcMotor.class, "intakeSlider");
        intakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        outtakeSliderUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderUp.setTargetPosition(0);
        outtakeSliderUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeSliderDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderDown.setTargetPosition(0);
        outtakeSliderDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeSliderUp.setPower(0);
        outtakeSliderDown.setPower(0);

        intakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlider.setTargetPosition(0);
        intakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeSlider.setPower(0);
    }

    //------------------------------IntakeSlider------------------------------//

    public void ExtendIntakeSlider(double power) {
        intakeSlider.setTargetPosition(RobotConstants.intakeSliderExtendPosition);
        intakeSlider.setPower(power);
    }

    public void RetractIntakeSlider(double power) {
        intakeSlider.setTargetPosition(RobotConstants.intakeSliderRetractPosition);
        intakeSlider.setPower(power);
    }

    public void StopIntakeSlider(){
        intakeSlider.setPower(0);
    }

    //------------------------------OuttakeSlider------------------------------//

    public void ExtendOuttakeSlider(double power) {
        outtakeSliderUp.setTargetPosition(RobotConstants.outtakeSliderExtendPosition);
        outtakeSliderDown.setTargetPosition(RobotConstants.outtakeSliderExtendPosition);
        outtakeSliderUp.setPower(power);
        outtakeSliderDown.setPower(power);
    }

    public void RetractOuttakeSlider(double power) {
        outtakeSliderUp.setTargetPosition(RobotConstants.outtakeSliderRetractPosition);
        outtakeSliderDown.setTargetPosition(RobotConstants.outtakeSliderRetractPosition);
        outtakeSliderUp.setPower(power);
        outtakeSliderDown.setPower(power);
    }

    public void StopOuttakeSlider(){
        outtakeSliderUp.setPower(0);
        outtakeSliderDown.setPower(0);
    }

    //------------------------------Getters------------------------------//

    public int getSliderIntakePosition() {
        return intakeSlider.getCurrentPosition();
    }

    public double getSliderIntakePower(){
        return  intakeSlider.getPower();
    }

    public int getSliderUpOuttakePosition() {
        return outtakeSliderUp.getCurrentPosition();
    }

    public int getSliderDownOuttakePosition() {
        return outtakeSliderDown.getCurrentPosition();
    }

    public double getSliderUpOuttakePower(){
        return  outtakeSliderUp.getPower();
    }

    public double getSliderDownOuttakePower(){
        return  outtakeSliderDown.getPower();
    }
}
