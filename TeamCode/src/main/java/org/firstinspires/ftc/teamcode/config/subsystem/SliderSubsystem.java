package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class SliderSubsystem {
    private final DcMotorEx outtakeSliderUp, outtakeSliderDown, intakeSlider;

    public SliderSubsystem(HardwareMap hardwareMap) {
        outtakeSliderUp = hardwareMap.get(DcMotorEx.class, "outtakeSliderUp");
        outtakeSliderUp.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSliderDown = hardwareMap.get(DcMotorEx.class, "outtakeSliderDown");
        outtakeSliderDown.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlider = hardwareMap.get(DcMotorEx.class, "intakeSlider");
        intakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //------------------------------IntakeSlider------------------------------//

    public void InitIntake(){
        intakeSlider.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlider.setTargetPosition(0);
        intakeSlider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        intakeSlider.setPower(0);
    }

    public void MoveIntake(int position, double power){
        intakeSlider.setTargetPosition(position);
        intakeSlider.setPower(power);
    }

    public void StopIntake(){
        intakeSlider.setPower(0);
    }

    //------------------------------OuttakeSlider------------------------------//

    public void InitOuttake(){
        outtakeSliderUp.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderUp.setTargetPosition(0);
        outtakeSliderUp.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        outtakeSliderDown.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderDown.setTargetPosition(0);
        outtakeSliderDown.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        outtakeSliderUp.setPower(0);
        outtakeSliderDown.setPower(0);
    }

    public void MoveOuttake(int position, double power){
        outtakeSliderUp.setTargetPosition(position);
        outtakeSliderDown.setTargetPosition(position);
        outtakeSliderUp.setPower(power);
        outtakeSliderDown.setPower(power);
    }

    public void StopOuttake(){
        outtakeSliderUp.setPower(0);
        outtakeSliderDown.setPower(0);
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
}
