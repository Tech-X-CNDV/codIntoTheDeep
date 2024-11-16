
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
    static final double axonMinPos = 0.3;
    static final double maxPosROTINT = 0.46;
    static final double maxOuttakeSliderSpeed = 1.0;
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
            outtakeSliderUp.setDirection(DcMotorSimple.Direction.REVERSE);
            
        } catch(IllegalArgumentException e){
            outtakeSliderDown = null;
            telemetry.addData("Error", "OuttakeSliderDown not found.");
        }
        try{
            intakeSlider = hardwareMap.get(DcMotor.class, "intakeSlider");
            intakeSlider.setDirection(DcMotorSimple.Direction.FORWARD);

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
            leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(IllegalArgumentException e){
            leftFrontMotor = null;
            telemetry.addData("Error", "LeftFrontMotor not found");
        }
        try{
            rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e){
            rightFrontMotor = null;
            telemetry.addData("Error", "RightFrontMotor not found");
        }
        try{
            leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
            leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(IllegalArgumentException e){
            leftRearMotor = null;
            telemetry.addData("Error", "LeftRearMotor not found");
        }
        try{
            rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
            rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
    }

    @Override
    public void start() {
        // Reset the runtime when the play button is pressed
        runtime.reset();
    }

    boolean changedOUT = false;
    boolean changedINT = false;
    boolean changedROTINT = false;
    boolean arePiesa = false;
    boolean speedLimit = false;
    boolean pressLbumper2 = false;
    boolean intakeAxonOn = false;
    boolean intakeAxonMove = false;
    double outtakeAxonVal = 0.5;

    public void loop() {
        if (robot.servoRotireInt != null){
            ToggleRotireIntake();
        }
        if(robot.servoRotireInt != null) {
            ToggleIntake();
        }
        if(robot.servoGhiaraOut != null) {
            ToggleOuttake();
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

    private void ToggleRotireIntake() {
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

    private void ToggleIntake() {
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

    private void ToggleOuttake() {
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
        robot.outtakeSliderUp.setPower(gamepad2.left_stick_y);
        robot.outtakeSliderDown.setPower(gamepad2.left_stick_y);
    }

    private void IntakeSliderMotion(){
        if(gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0){
            robot.intakeSlider.setPower(gamepad1.right_trigger);
        } else{
            robot.intakeSlider.setPower(-gamepad1.left_trigger);
        }
    }

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

    private void OuttakeAxonMotion(){
        if(gamepad2.right_stick_y < 0 && outtakeAxonVal < 1){
            outtakeAxonVal += 0.01;
        }
        if(gamepad2.right_stick_y > 0 && outtakeAxonVal > 0){
            outtakeAxonVal -= 0.01;
        }
        if(gamepad2.dpad_up){
            outtakeAxonVal = 1;
        }
        if(gamepad2.dpad_down){
            outtakeAxonVal = 0;
        }
        if(gamepad2.dpad_right){
            outtakeAxonVal = 0.3;
        }
        robot.outtakeAxonRight.setPosition(outtakeAxonVal);
        robot.outtakeAxonLeft.setPosition(outtakeAxonVal);
    }

    private void IntakeAxonMotion(){
        if(gamepad1.y && intakeAxonMove){
            robot.intakeAxonLeft.setPosition(intakeAxonOn ? 0 : 0.21);
            robot.intakeAxonRight.setPosition(intakeAxonOn ? 0 : 0.21);
            intakeAxonOn = !intakeAxonOn;
            intakeAxonMove = false;
        } else if(!gamepad1.y){
            intakeAxonMove = true;
        }
    }

    private void Roti() {
        if (gamepad2.left_bumper && pressLbumper2) {
            speedLimit = !speedLimit;
            pressLbumper2 = false;
        }
        if (!gamepad2.left_bumper) {
            pressLbumper2 = true;
        }

        double drive = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double turn = gamepad2.right_stick_x;

        // Apply speed limit if active
        if (speedLimit) {
            drive /= 2.0d;
            strafe /= 2.0d;
            turn /= 2.0d;
        }

        // Reduce overall speed based on right trigger
        double speedMultiplier = 1.0 - 0.75d * this.gamepad2.right_trigger;
        double max;

        // Calculate individual motor powers (adjust signs as needed)
        double frontLeftPower = (drive + strafe + turn) * speedMultiplier;
        double frontRightPower = (drive - strafe - turn) * speedMultiplier;
        double rearLeftPower = (drive - strafe + turn) * speedMultiplier;
        double rearRightPower = (drive + strafe - turn) * speedMultiplier;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(rearLeftPower));
        max = Math.max(max, Math.abs(rearRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            rearLeftPower /= max;
            rearRightPower /= max;
        }
        robot.leftFrontMotor.setPower(frontLeftPower);
        robot.rightFrontMotor.setPower(frontRightPower);
        robot.leftRearMotor.setPower(rearLeftPower);
        robot.rightRearMotor.setPower(rearRightPower);
    }

    private void Telemetry() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
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
        }
        if(robot.intakeSlider != null){
            telemetry.addData("IntakeSliderPower", robot.intakeSlider.getPower());
        }
        telemetry.update();
    }

}