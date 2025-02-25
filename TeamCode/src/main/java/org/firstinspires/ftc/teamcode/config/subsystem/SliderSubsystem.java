package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class SliderSubsystem {
    private final DcMotorEx outtakeSliderUp, outtakeSliderDown, intakeSlider;
    boolean outtakeResseted = false, intakeResseted = false;

    public SliderSubsystem(HardwareMap hardwareMap) {
        outtakeSliderUp = hardwareMap.get(DcMotorEx.class, "outtakeSliderUp");
        outtakeSliderUp.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeSliderUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeSliderDown = hardwareMap.get(DcMotorEx.class, "outtakeSliderDown");
        outtakeSliderDown.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSliderDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeSlider = hardwareMap.get(DcMotorEx.class, "intakeSlider");
        intakeSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //------------------------------IntakeSlider------------------------------//

    public void InitIntake(boolean auto){
        intakeSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlider.setTargetPosition(0);
        intakeSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        intakeSlider.setPower(0);
        if(auto)
            intakeResseted = true;
    }

    public void MoveIntake(int position, double power){
        if(intakeResseted) {
            intakeSlider.setTargetPosition(position);
            intakeSlider.setPower(power);
        }
    }

    public void StopIntake(){
        if(intakeResseted)
            intakeSlider.setPower(0);
    }

    public void ResetIntake(){
        intakeSlider.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlider.setPower(-0.5);
    }

    public void ResetIntakeEncoder(){
        intakeSlider.setPower(0);
        intakeSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlider.setTargetPosition(0);
        intakeSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        intakeResseted = true;
    }

    //------------------------------OuttakeSlider------------------------------//

    public void InitOuttake(boolean auto){
        outtakeSliderUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderUp.setTargetPosition(0);
        outtakeSliderUp.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        outtakeSliderDown.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderDown.setTargetPosition(0);
        outtakeSliderDown.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        outtakeSliderUp.setPower(0);
        outtakeSliderDown.setPower(0);
        if(auto)
            outtakeResseted = true;
    }

    public void MoveOuttake(int position, double power){
        if(outtakeResseted) {
            outtakeSliderUp.setTargetPosition(position);
            outtakeSliderDown.setTargetPosition(position);
            outtakeSliderUp.setPower(power);
            outtakeSliderDown.setPower(power);
        }
    }

    public void StopOuttake(){
        if(outtakeResseted) {
            outtakeSliderUp.setPower(0);
            outtakeSliderDown.setPower(0);
        }
    }

    public void ResetOuttake(){
        outtakeSliderUp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSliderDown.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSliderUp.setPower(-0.5);
        outtakeSliderDown.setPower(-0.5);
    }

    public void ResetOuttakeEncoder(){
        outtakeSliderUp.setPower(0);
        outtakeSliderDown.setPower(0);
        outtakeSliderUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderDown.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderDown.setTargetPosition(0);
        outtakeSliderUp.setTargetPosition(0);
        outtakeSliderUp.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeSliderDown.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        outtakeResseted = true;
    }

    //------------------------------Getters------------------------------//

    public int GetIntakePosition() {
        return intakeSlider.getCurrentPosition();
    }

    public double GetIntakePower(){
        return  intakeSlider.getPower();
    }

    public double GetIntakeAmperage(){
        return intakeSlider.getCurrent(CurrentUnit.AMPS);
    }

    public int GetUpOuttakePosition() {
        return outtakeSliderUp.getCurrentPosition();
    }

    public int GetDownOuttakePosition() {
        return outtakeSliderDown.getCurrentPosition();
    }

    public double GetUpOuttakePower(){
        return  outtakeSliderUp.getPower();
    }

    public double GetDownOuttakePower(){
        return  outtakeSliderDown.getPower();
    }

    public double GetOuttakeAmperage(){
        return (outtakeSliderUp.getCurrent(CurrentUnit.AMPS) + outtakeSliderDown.getCurrent(CurrentUnit.AMPS)) / 2;
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
