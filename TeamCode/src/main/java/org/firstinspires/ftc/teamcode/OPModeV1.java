
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

class CRobo {

    // Servouri
    public Servo servoGhiaraOut = null;
    public Servo servoGhiaraInt = null;
    public Servo servoRotireInt = null;
    public Servo intakeAxonLeft = null;
    public Servo intakeAxonRight = null;
    public Servo outtakeAxonLeft = null;
    public Servo outtakeAxonRight = null;
    // Senzori
    public DistanceSensor dSensor = null;
    // Variabile
    static final double maxPos = 0.3;
    static final double maxPosINT = 0.5;
    static final double minPos = 0.0;
    static final double axonMinPos = 0.4;
    static final double maxPosROTINT = 0.46;
    static final int outtakeSliderExtendPosition = 4150;
    static final int outtakeSliderRetractPosition = 0;
    static final int intakeSliderExtendPosition = 680;
    static final int intakeSliderRetractPosition = 0;
    // Motoare
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    public DcMotor outtakeSliderUp = null;
    public DcMotor outtakeSliderDown = null;
    public DcMotor intakeSlider = null;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        // Initializare servo si senzor
        try {
            servoGhiaraOut = hardwareMap.get(Servo.class, "ghiaraOuttake");
            servoGhiaraOut.setPosition(maxPos);
        } catch (IllegalArgumentException e) {
            servoGhiaraOut = null;
            telemetry.addData("Error", "servoGhiaraOut not found.");
        }
        try{
            servoGhiaraInt = hardwareMap.get(Servo.class, "ghiaraIntake");
            servoGhiaraInt.setPosition(minPos);
        } catch(IllegalArgumentException e){
            servoGhiaraInt = null;
            telemetry.addData("Error", "servoGhiaraInt not found.");
        }
        try {
            servoRotireInt = hardwareMap.get(Servo.class, "rotireGhiaraIntake");
            servoRotireInt.setPosition(minPos);
        } catch(IllegalArgumentException e) {
            servoRotireInt = null;
            telemetry.addData("Error", "servoRotireInt not found.");
        }
        try{
            intakeAxonLeft = hardwareMap.get(Servo.class, "intakeAxonLeft");
            intakeAxonLeft.setPosition(minPos);
        } catch(IllegalArgumentException e){
            intakeAxonLeft = null;
            telemetry.addData("Error", "IntakeAxonLeft not found.");
        }
        try{
            intakeAxonRight = hardwareMap.get(Servo.class, "intakeAxonRight");
            intakeAxonRight.setPosition(minPos);
        } catch(IllegalArgumentException e){
            intakeAxonRight = null;
            telemetry.addData("Error", "IntakeAxonRight not found.");
        }
        try{
            outtakeAxonLeft = hardwareMap.get(Servo.class, "outtakeAxonLeft");
            outtakeAxonLeft.setPosition(axonMinPos);
        } catch(IllegalArgumentException e){
            outtakeAxonLeft = null;
            telemetry.addData("Error", "OuttakeAxonLeft not found.");
        }
        try{
            outtakeAxonRight = hardwareMap.get(Servo.class, "outtakeAxonRight");
            outtakeAxonRight.setPosition(axonMinPos);
        } catch(IllegalArgumentException e){
            outtakeAxonRight = null;
            telemetry.addData("Error", "OuttakeAxonRight not found.");
        }
        try{
            outtakeSliderUp = hardwareMap.get(DcMotor.class, "outtakeSliderUp");
            outtakeSliderUp.setDirection(DcMotorSimple.Direction.FORWARD);
            
        } catch(IllegalArgumentException e){
            outtakeSliderUp = null;
            telemetry.addData("Error", "OuttakeSliderUp not found.");
        }
        try{
            outtakeSliderDown = hardwareMap.get(DcMotor.class, "outtakeSliderDown");
            outtakeSliderDown.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e){
            outtakeSliderDown = null;
            telemetry.addData("Error", "OuttakeSliderDown not found.");
        }
        try{
            intakeSlider = hardwareMap.get(DcMotor.class, "intakeSlider");
            intakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        } catch(IllegalArgumentException e){
            intakeSlider = null;
            telemetry.addData("Error", "IntakeSlider not found.");
        }
        try{
            dSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        } catch(IllegalArgumentException e){
            dSensor = null;
            telemetry.addData("Error", "Distance Sensor not found");
        }
        // Initializare roti
        try{
            leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e){
            leftFrontMotor = null;
            telemetry.addData("Error", "LeftFrontMotor not found");
        }
        try{
            rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
            rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(IllegalArgumentException e){
            rightFrontMotor = null;
            telemetry.addData("Error", "RightFrontMotor not found");
        }
        try{
            leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
            leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e){
            leftRearMotor = null;
            telemetry.addData("Error", "LeftRearMotor not found");
        }
        try{
            rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
            rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(IllegalArgumentException e){
            rightRearMotor = null;
            telemetry.addData("Error", "RightRearMotor not found");
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}

@TeleOp(name = "Basic: OPModeV1", group = "Linear OpMode")
public class OPModeV1 extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();

    CRobo robot = new CRobo();

    @Override
    public void init() {
        robot.init(telemetry, hardwareMap);

        robot.outtakeSliderUp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeSliderUp.setTargetPosition(0);
        robot.outtakeSliderUp.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.outtakeSliderDown.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.outtakeSliderDown.setTargetPosition(0);
        robot.outtakeSliderDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.outtakeSliderUp.setPower(0);
        robot.outtakeSliderDown.setPower(0);

        robot.intakeSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.intakeSlider.setTargetPosition(0);
        robot.intakeSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.intakeSlider.setPower(0);
    }

    @Override
    public void start() {
        // Reset the runtime when the play button is pressed
        runtime.reset();
    }

    public void loop() {
        if (robot.servoRotireInt != null){
            ToggleGhiaraRotireIntake();
        }
        if(robot.servoGhiaraInt != null) {
            ToggleGhiaraIntake();
        }
        if(robot.servoGhiaraOut != null) {
            ToggleGhiaraOuttake();
        }
        if(robot.dSensor != null){
            AutoPrindere();
        }
        if(robot.outtakeAxonLeft != null && robot.outtakeAxonRight != null){
            OuttakeAxonMotion();
        }
        if(robot.intakeAxonLeft != null && robot.intakeAxonRight != null){
            IntakeAxonMotion();
        }
        if(robot.outtakeSliderUp != null && robot.outtakeSliderDown != null){
            OuttakeSliderMotion();
        }
        if(robot.intakeSlider != null){
            IntakeSliderMotion();
        }
        if(robot.leftFrontMotor != null && robot.rightFrontMotor != null && robot.leftRearMotor != null && robot.rightRearMotor != null) {
            Roti();
        }
        Telemetry();
    }

    boolean changedROTINT = false;

    private void ToggleGhiaraRotireIntake() {
        if (gamepad1.b && !changedROTINT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoRotireInt
                    .setPosition(robot.servoRotireInt.getPosition() == 0 ? CRobo.maxPosROTINT : CRobo.minPos);
            changedROTINT = true;
        }else if(!gamepad1.b){
            changedROTINT = false;
        }
    }

    boolean changedINT = false;

    private void ToggleGhiaraIntake() {
        if (gamepad1.a && !changedINT) {
            arePiesa = false;
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoGhiaraInt.setPosition(robot.servoGhiaraInt.getPosition() == 0 ? CRobo.maxPosINT : CRobo.minPos);
            changedINT = true;
        } else if (!gamepad1.a) {
            changedINT = false;
        }
    }

    boolean changedOUT = false;

    private void ToggleGhiaraOuttake() {
        if (gamepad2.a && !changedOUT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoGhiaraOut.setPosition(robot.servoGhiaraOut.getPosition() == 0 ? CRobo.maxPos : CRobo.minPos);
            changedOUT = true;
        } else if (!gamepad2.a) {
            changedOUT = false;
        }
    }

    private void OuttakeSliderMotion(){
        if(gamepad2.left_stick_y < 0.0){
            robot.outtakeSliderUp.setTargetPosition(CRobo.outtakeSliderExtendPosition);
            robot.outtakeSliderDown.setTargetPosition(CRobo.outtakeSliderExtendPosition);
            robot.outtakeSliderUp.setPower(-gamepad2.left_stick_y);
            robot.outtakeSliderDown.setPower(-gamepad2.left_stick_y);
        }
        if(gamepad2.left_stick_y > 0.0){
            robot.outtakeSliderUp.setTargetPosition(CRobo.outtakeSliderRetractPosition);
            robot.outtakeSliderDown.setTargetPosition(CRobo.outtakeSliderRetractPosition);
            robot.outtakeSliderUp.setPower(gamepad2.left_stick_y);
            robot.outtakeSliderDown.setPower(gamepad2.left_stick_y);
        }
        if(gamepad2.left_stick_y == 0.0){
            robot.outtakeSliderUp.setPower(0.0);
            robot.outtakeSliderDown.setPower(0.0);
        }

    }

    private void IntakeSliderMotion(){
        if(gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0){
            robot.intakeSlider.setTargetPosition(CRobo.intakeSliderExtendPosition);
            robot.intakeSlider.setPower(gamepad1.right_trigger);
        } else if(gamepad1.left_trigger != 0){
            robot.intakeSlider.setTargetPosition(CRobo.intakeSliderRetractPosition);
            robot.intakeSlider.setPower(gamepad1.left_trigger);
        } else {
            robot.intakeSlider.setPower(0.0);
        }
    }

    boolean arePiesa = false;

    private void AutoPrindere() {
        double val = robot.dSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", val);

        if (val < 3.7) {
            robot.servoGhiaraInt.setPosition(0);
            arePiesa = true;
            if (gamepad1.a) {
                robot.servoGhiaraInt.setPosition(CRobo.maxPosINT);
            }
        }
    }

    double outtakeAxonVal = 0.4;

    private void OuttakeAxonMotion(){
        if(gamepad2.right_stick_y < 0 && outtakeAxonVal < 1){
            outtakeAxonVal += 0.005;
        }
        if(gamepad2.right_stick_y > 0 && outtakeAxonVal > 0){
            outtakeAxonVal -= 0.005;
        }
        if(gamepad2.dpad_up){
            outtakeAxonVal = 1;
        }
        if(gamepad2.dpad_down){
            outtakeAxonVal = 0.1;
        }
        if(gamepad2.dpad_right){
            outtakeAxonVal = 0.4;
        }
        robot.outtakeAxonRight.setPosition(outtakeAxonVal);
        robot.outtakeAxonLeft.setPosition(outtakeAxonVal);
    }

    boolean intakeAxonOn = false;
    boolean intakeAxonMove = false;
    boolean intakeMidAxonMove = false;
    boolean intakeMidAxonOn = false;

    private void IntakeAxonMotion(){
        if(gamepad1.x && intakeMidAxonMove && !intakeAxonOn){
            robot.intakeAxonLeft.setPosition(intakeMidAxonOn ? 0 : 0.20);
            robot.intakeAxonRight.setPosition(intakeMidAxonOn ? 0 : 0.20);
            intakeMidAxonOn = !intakeMidAxonOn;
            intakeMidAxonMove = false;
        }else if(!gamepad1.x){
            intakeMidAxonMove = true;
        }
        if(gamepad1.y && intakeAxonMove && intakeMidAxonOn){
            robot.intakeAxonLeft.setPosition(intakeAxonOn ? 0.20 : 0.27);
            robot.intakeAxonRight.setPosition(intakeAxonOn ? 0.20 : 0.27);
            intakeAxonOn = !intakeAxonOn;
            intakeAxonMove = false;
        } else if(!gamepad1.y){
            intakeAxonMove = true;
        }
    }

    double maxMotorSpeed = 0.0;
    double frontLeftPower = 0.0;
    double rearLeftPower = 0.0;
    double frontRightPower = 0.0;
    double rearRightPower = 0.0;
    double driveMotor = 0.0;
    double strafeMotor = 0.0;
    double turnMotor = 0.0;
    boolean speedLimit = false;
    boolean pressLbumper2 = false;

    private void Roti() {
        if (gamepad1.left_bumper && pressLbumper2) {
            speedLimit = !speedLimit;
            pressLbumper2 = false;
        } else if (!gamepad1.left_bumper) {
            pressLbumper2 = true;
        }

        driveMotor = -gamepad1.left_stick_y;
        strafeMotor = gamepad1.left_stick_x;
        turnMotor = gamepad1.right_stick_x;

        // Apply speed limit if active
        if (speedLimit) {
            driveMotor /= 2.0d;
            strafeMotor /= 2.0d;
            turnMotor /= 2.0d;
        }

        // Calculate individual motor powers (adjust signs as needed)
        frontLeftPower = (driveMotor + strafeMotor + turnMotor);
        frontRightPower = (driveMotor - strafeMotor - turnMotor);
        rearLeftPower = (driveMotor - strafeMotor + turnMotor);
        rearRightPower = (driveMotor + strafeMotor - turnMotor);

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        maxMotorSpeed = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxMotorSpeed = Math.max(maxMotorSpeed, Math.abs(rearLeftPower));
        maxMotorSpeed = Math.max(maxMotorSpeed, Math.abs(rearRightPower));

        if (maxMotorSpeed > 1.0) {
            frontLeftPower /= maxMotorSpeed;
            frontRightPower /= maxMotorSpeed;
            rearLeftPower /= maxMotorSpeed;
            rearRightPower /= maxMotorSpeed;
        }
        robot.leftFrontMotor.setPower(frontLeftPower);
        robot.rightFrontMotor.setPower(frontRightPower);
        robot.leftRearMotor.setPower(rearLeftPower);
        robot.rightRearMotor.setPower(rearRightPower);
    }

    private void Telemetry() {
        telemetry.addData("Status", "Run Time: " + String.format(Locale.US, "%d:%.2f", (int)(runtime.seconds() / 60), runtime.seconds() % 60));
        if(speedLimit){
            telemetry.addData("Motor", "SpeedLimit enabled.");
        }
        if (robot.servoGhiaraInt != null) {
            double ghiaraintPos = robot.servoGhiaraInt.getPosition();
            double ghiaraintangle = ghiaraintPos * 180;
            telemetry.addData("Intake Position", ghiaraintPos);
            telemetry.addData("Intake Angle", ghiaraintangle);
        }
        if (robot.servoGhiaraOut != null){
            double ghiaraoutPos = robot.servoGhiaraOut.getPosition();
            double ghiaraoutangle = ghiaraoutPos * 180;
            telemetry.addData("Outtake Position", ghiaraoutPos);
            telemetry.addData("Outtake Angle", ghiaraoutangle);
        }
        if(robot.servoRotireInt != null){
            double servortireintPos = robot.servoRotireInt.getPosition();
            double servorotireangle = servortireintPos * 180;
            telemetry.addData("Intake Position", servortireintPos);
            telemetry.addData("Intake Rotation Angle", servorotireangle);
        }
        if(robot.intakeAxonLeft != null && robot.intakeAxonRight != null){
            double intakeAxonLeftPos = robot.intakeAxonLeft.getPosition();
            double intakeAxonLeftAngle = intakeAxonLeftPos * 180;
            double intakeAxonRightPos = robot.intakeAxonRight.getPosition();
            double intakeAxonRightAngle = intakeAxonRightPos * 180;
            telemetry.addData("AxonIntakeLeft Position", intakeAxonLeftPos);
            telemetry.addData("AxonIntakeLeft Rotation Angle", intakeAxonLeftAngle);
            telemetry.addData("AxonIntakeRight Position", intakeAxonRightPos);
            telemetry.addData("AxonIntakeRight Rotation Angle", intakeAxonRightAngle);
        }
        if(robot.outtakeAxonLeft != null && robot.outtakeAxonRight != null){
            double outtakeAxonLeftPos = robot.outtakeAxonLeft.getPosition();
            double outtakeAxonLeftAngle = outtakeAxonLeftPos * 180;
            double outtakeAxonRightPos = robot.outtakeAxonRight.getPosition();
            double outtakeAxonRightAngle = outtakeAxonRightPos * 180;
            telemetry.addData("AxonOuttakeLeft Position", outtakeAxonLeftPos);
            telemetry.addData("AxonOuttakeLeft Rotation Angle", outtakeAxonLeftAngle);
            telemetry.addData("AxonOuttakeRight Position", outtakeAxonRightPos);
            telemetry.addData("AxonOuttakeRight Rotation Angle", outtakeAxonRightAngle);
        }
        if(robot.outtakeSliderUp != null && robot.outtakeSliderDown != null){
            telemetry.addData("OuttakeSliderPowerUp", robot.outtakeSliderUp.getPower());
            telemetry.addData("OuttakeSliderPowerDown", robot.outtakeSliderDown.getPower());
            telemetry.addData("OuttakeSliderPositionUp", robot.outtakeSliderUp.getCurrentPosition());
            telemetry.addData("OuttakeSliderPositionDown", robot.outtakeSliderDown.getCurrentPosition());
        }
        if(robot.intakeSlider != null){
            telemetry.addData("IntakeSliderPower", robot.intakeSlider.getPower());
            telemetry.addData("IntakeSliderPos", robot.intakeSlider.getCurrentPosition());
        }
        telemetry.update();
    }

}