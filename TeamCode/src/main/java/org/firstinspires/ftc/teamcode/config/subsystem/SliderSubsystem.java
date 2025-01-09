package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class SliderSubsystem {
    private final DcMotor outtakeSliderUp, outtakeSliderDown, intakeSlider;

    public SliderSubsystem(HardwareMap hardwareMap) {
        outtakeSliderUp = hardwareMap.get(DcMotor.class, "outtakeSliderUp");
        outtakeSliderUp.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeSliderDown = hardwareMap.get(DcMotor.class, "outtakeSliderDown");
        outtakeSliderDown.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeSlider = hardwareMap.get(DcMotor.class, "intakeSlider");
        intakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //------------------------------IntakeSlider------------------------------//

    public void InitIntake(){
        intakeSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeSlider.setPower(-0.1); // temp variable
    }

    public void StartIntake(){
        intakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlider.setTargetPosition(0);
        intakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        outtakeSliderUp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSliderDown.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeSliderUp.setPower(-0.1); // temp variable
        outtakeSliderDown.setPower(-0.1); // temp variable
    }

    public void StartOuttake(){
        outtakeSliderUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderUp.setTargetPosition(0);
        outtakeSliderUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        outtakeSliderDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSliderDown.setTargetPosition(0);
        outtakeSliderDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
}
