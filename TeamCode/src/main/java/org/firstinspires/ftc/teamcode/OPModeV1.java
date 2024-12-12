
package org.firstinspires.ftc.teamcode;


import static java.lang.Thread.sleep;

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
    static final double maxPosINT = 0.42;
    static final double minPos = 0.0;
    static final double axonMinPos = 0.4;
    static final double maxPosROTINT = 0.46;
    static final double intakeUpPos = 0;
    static final double intakeMiddlePos = 0.22;
    static final double intakeDownPos = 0.25;
    static final double outtakeUpPos = 1;
    static final double outtakeMidPos = 0.4;
    static final double outtakeBehindPos = 0.1;
    static final int outtakeSliderExtendPosition = 4160;
    static final int outtakeSliderRetractPosition = 0;
    static final int intakeSliderExtendPosition = 650;
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
            servoGhiaraOut.setPosition(minPos);
        } catch (IllegalArgumentException e) {
            servoGhiaraOut = null;
            telemetry.addData("Error", "servoGhiaraOut not found.");
        }
        try {
            servoGhiaraInt = hardwareMap.get(Servo.class, "ghiaraIntake");
            servoGhiaraInt.setPosition(minPos);
        } catch(IllegalArgumentException e) {
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
        try {
            intakeAxonLeft = hardwareMap.get(Servo.class, "intakeAxonLeft");
            intakeAxonLeft.setPosition(minPos);
        } catch(IllegalArgumentException e) {
            intakeAxonLeft = null;
            telemetry.addData("Error", "IntakeAxonLeft not found.");
        }
        try {
            intakeAxonRight = hardwareMap.get(Servo.class, "intakeAxonRight");
            intakeAxonRight.setPosition(minPos);
        } catch(IllegalArgumentException e) {
            intakeAxonRight = null;
            telemetry.addData("Error", "IntakeAxonRight not found.");
        }
        try {
            outtakeAxonLeft = hardwareMap.get(Servo.class, "outtakeAxonLeft");
            outtakeAxonLeft.setPosition(axonMinPos);
        } catch(IllegalArgumentException e) {
            outtakeAxonLeft = null;
            telemetry.addData("Error", "OuttakeAxonLeft not found.");
        }
        try {
            outtakeAxonRight = hardwareMap.get(Servo.class, "outtakeAxonRight");
            outtakeAxonRight.setPosition(axonMinPos);
        } catch(IllegalArgumentException e) {
            outtakeAxonRight = null;
            telemetry.addData("Error", "OuttakeAxonRight not found.");
        }
        try {
            outtakeSliderUp = hardwareMap.get(DcMotor.class, "outtakeSliderUp");
            outtakeSliderUp.setDirection(DcMotorSimple.Direction.FORWARD);
            
        } catch(IllegalArgumentException e) {
            outtakeSliderUp = null;
            telemetry.addData("Error", "OuttakeSliderUp not found.");
        }
        try {
            outtakeSliderDown = hardwareMap.get(DcMotor.class, "outtakeSliderDown");
            outtakeSliderDown.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e) {
            outtakeSliderDown = null;
            telemetry.addData("Error", "OuttakeSliderDown not found.");
        }
        try {
            intakeSlider = hardwareMap.get(DcMotor.class, "intakeSlider");
            intakeSlider.setDirection(DcMotorSimple.Direction.REVERSE);

        } catch(IllegalArgumentException e) {
            intakeSlider = null;
            telemetry.addData("Error", "IntakeSlider not found.");
        }
        try {
            dSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        } catch(IllegalArgumentException e) {
            dSensor = null;
            telemetry.addData("Error", "Distance Sensor not found");
        }
        // Initializare roti
        try {
            leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e) {
            leftFrontMotor = null;
            telemetry.addData("Error", "LeftFrontMotor not found");
        }
        try {
            rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
            rightFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(IllegalArgumentException e) {
            rightFrontMotor = null;
            telemetry.addData("Error", "RightFrontMotor not found");
        }
        try {
            leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
            leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch(IllegalArgumentException e) {
            leftRearMotor = null;
            telemetry.addData("Error", "LeftRearMotor not found");
        }
        try {
            rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
            rightRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch(IllegalArgumentException e) {
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
        if (robot.servoRotireInt != null) {
            ToggleGhiaraRotireIntake();
        }
        if(robot.servoGhiaraInt != null) {
            ToggleGhiaraIntake();
        }
        if(robot.servoGhiaraOut != null) {
            ToggleGhiaraOuttake();
        }
        if(robot.dSensor != null) {
            AutoPrindere();
        }
        if(robot.outtakeAxonLeft != null && robot.outtakeAxonRight != null) {
            OuttakeAxonMotion();
        }
        if(robot.intakeAxonLeft != null && robot.intakeAxonRight != null) {
            IntakeAxonMotion();
        }
        if(robot.outtakeSliderUp != null && robot.outtakeSliderDown != null) {
            OuttakeSliderMotion();
        }
        if(robot.intakeSlider != null) {
            IntakeSliderMotion();
        }
        if(robot.leftFrontMotor != null && robot.rightFrontMotor != null && robot.leftRearMotor != null && robot.rightRearMotor != null) {
            Roti();
        }
        Telemetry();
    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    boolean outtakeAutoSlider = false;

    public void OuttakeSlidersMotionAction(double power,int position, boolean auto){
        if(auto) {
            outtakeAutoSlider = true;
        }
        robot.outtakeSliderDown.setTargetPosition(position);
        robot.outtakeSliderUp.setTargetPosition(position);
        robot.outtakeSliderUp.setPower(power);
        robot.outtakeSliderDown.setPower(power);
    }

    boolean intakeAutoSlider = false;

    public void IntakeSlidersMotionAction(double power, int position, boolean auto){
        if(auto) {
            intakeAutoSlider = true;
        }
        robot.intakeSlider.setTargetPosition(position);
        robot.intakeSlider.setPower(power);
    }

    public void IntakeToOuttake() {
        sleep(200);
        robot.intakeAxonLeft.setPosition(CRobo.intakeUpPos);
        robot.intakeAxonRight.setPosition(CRobo.intakeUpPos);
        robot.servoRotireInt.setPosition(CRobo.minPos);
        intakeMidAxonOn = false;
        intakeAxonOn = false;
        sleep(200);
        robot.servoGhiaraInt.setPosition(CRobo.maxPosINT);
        sleep(400);
        IntakeSlidersMotionAction(1, CRobo.intakeSliderRetractPosition, true);
        robot.intakeAxonLeft.setPosition(CRobo.intakeMiddlePos);
        robot.intakeAxonRight.setPosition(CRobo.intakeMiddlePos);
        intakeMidAxonOn = true;
    }
    boolean changedROTINT = false;

    private void ToggleGhiaraRotireIntake() {
        if (gamepad1.b && !changedROTINT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoRotireInt
                    .setPosition(robot.servoRotireInt.getPosition() == 0 ? CRobo.maxPosROTINT : CRobo.minPos);
            changedROTINT = true;
        }else if(!gamepad1.b) {
            changedROTINT = false;
        }
    }

    boolean changedINT = false;

    private void ToggleGhiaraIntake() {
        if (gamepad1.a && !changedINT) {
            arePiesa = false;
            changedINT = true;
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoGhiaraInt.setPosition(robot.servoGhiaraInt.getPosition() == 0 ? CRobo.maxPosINT : CRobo.minPos);
            if(intakeAxonOn) {
                IntakeToOuttake();
            }
        } else if (!gamepad1.a) {
            changedINT = false;
        }
    }

    boolean changedOUT = false;

    private void ToggleGhiaraOuttake() {
        if ((gamepad2.a || gamepad2.right_bumper) && !changedOUT) {
            changedOUT = true;
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoGhiaraOut.setPosition(robot.servoGhiaraOut.getPosition() == 0 ? CRobo.maxPos : CRobo.minPos);
            if(robot.outtakeSliderUp.getCurrentPosition() > CRobo.outtakeSliderExtendPosition - 100) {
                sleep(300);
                outtakeAxonVal = CRobo.outtakeMidPos;
                OuttakeSlidersMotionAction(1, CRobo.outtakeSliderRetractPosition, true);
            }
        } else if (!gamepad2.a) {
            changedOUT = false;
        }
    }

    private void OuttakeSliderMotion(){
        if (gamepad2.left_stick_y != 0.0) {
            outtakeAutoSlider = false;
            OuttakeSlidersMotionAction(Math.abs(gamepad2.left_stick_y),
                    gamepad2.left_stick_y < 0 ? CRobo.outtakeSliderExtendPosition : CRobo.outtakeSliderRetractPosition, false);
        } else if (!outtakeAutoSlider) {
            robot.outtakeSliderUp.setPower(0.0);
            robot.outtakeSliderDown.setPower(0.0);
        }
    }

    private void IntakeSliderMotion(){
        if(gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0) {
            intakeAutoSlider = false;
            IntakeSlidersMotionAction(gamepad1.right_trigger, CRobo.intakeSliderExtendPosition, false);
        } else if(gamepad1.left_trigger != 0) {
            intakeAutoSlider = false;
            IntakeSlidersMotionAction(gamepad1.left_trigger, CRobo.intakeSliderRetractPosition, false);
        } else if(!intakeAutoSlider) {
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
        if (gamepad2.right_stick_y < 0 && outtakeAxonVal < 1) {
            outtakeAxonVal += 0.005;
        } else if (gamepad2.right_stick_y > 0 && outtakeAxonVal > 0) {
            outtakeAxonVal -= 0.005;
        } else if (gamepad2.dpad_up) {
            outtakeAxonVal = CRobo.outtakeUpPos;
            if (robot.servoGhiaraOut.getPosition() == CRobo.minPos) {
                OuttakeSlidersMotionAction(1, CRobo.outtakeSliderExtendPosition, true);
            }
        } else if (gamepad2.dpad_down) {
            outtakeAxonVal = CRobo.outtakeMidPos;
        } else if (gamepad2.dpad_left) {
            outtakeAxonVal = CRobo.outtakeBehindPos;
        }
        robot.outtakeAxonRight.setPosition(outtakeAxonVal);
        robot.outtakeAxonLeft.setPosition(outtakeAxonVal);
    }

    boolean intakeAxonOn = false;
    boolean intakeAxonMove = false;
    boolean intakeMidAxonMove = false;
    boolean intakeMidAxonOn = false;

    private void IntakeAxonMotion(){
        if(gamepad1.x && intakeMidAxonMove && !intakeAxonOn) {
            robot.intakeAxonLeft.setPosition(intakeMidAxonOn ? CRobo.intakeUpPos : CRobo.intakeMiddlePos);
            robot.intakeAxonRight.setPosition(intakeMidAxonOn ? CRobo.intakeUpPos : CRobo.intakeMiddlePos);
            intakeMidAxonOn = !intakeMidAxonOn;
            intakeMidAxonMove = false;
        }else if(!gamepad1.x) {
            intakeMidAxonMove = true;
        }
        if(gamepad1.y && intakeAxonMove && intakeMidAxonOn) {
            robot.intakeAxonLeft.setPosition(intakeAxonOn ? CRobo.intakeMiddlePos : CRobo.intakeDownPos);
            robot.intakeAxonRight.setPosition(intakeAxonOn ? CRobo.intakeMiddlePos : CRobo.intakeDownPos);
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