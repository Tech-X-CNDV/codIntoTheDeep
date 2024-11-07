
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
    // Senzori
    public DistanceSensor dSensor = null;
    // Variabile
    static final double maxPos = 0.19;
    static final double maxPosINT = 0.5;
    static final double minPos = 0.0;
    static final double maxPosROTINT = 0.5;
    // Motoare
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
        // Initializare servo si senzor
        try {
            servoGhiaraOut = hardwareMap.get(Servo.class, "GHIARAOUTTAKE");
            servoGhiaraOut.setPosition(minPos);
        } catch (IllegalArgumentException e) {
            servoGhiaraOut = null;
            telemetry.addData("Error", "servoGhiaraOut not found.");
        }
        try{
            servoGhiaraInt = hardwareMap.get(Servo.class, "GHIARAINTAKE");
            servoGhiaraInt.setPosition(minPos);
        } catch(IllegalArgumentException e){
            servoGhiaraInt = null;
            telemetry.addData("Error", "servoGhiaraInt not found.");
        }
        try {
            servoRotireInt = hardwareMap.get(Servo.class, "ROTIREGHIARAINTAKE");
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
            dSensor = hardwareMap.get(DistanceSensor.class, "DISTANCESENSOR");
        } catch(IllegalArgumentException e){
            dSensor = null;
            telemetry.addData("Error", "Distance Sensor not found");
        }
        // Initializare roti
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);

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
        if(robot.intakeAxonLeft != null && robot.intakeAxonRight != null){
            IntakeAxonMotion();
        }
        Roti();
        Telemetry();
    }

    private void ToggleRotireIntake() {
        if (gamepad2.b && !changedROTINT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoRotireInt
                    .setPosition(robot.servoRotireInt.getPosition() == 0 ? CRobo.maxPosROTINT : CRobo.minPos);
            changedROTINT = true;
        }else if(!gamepad2.b){
            changedROTINT = false;
        }
    }

    private void ToggleIntake() {
        if (gamepad2.a && !changedINT) {
            arePiesa = false;
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoGhiaraInt.setPosition(robot.servoGhiaraInt.getPosition() == 0 ? CRobo.maxPosINT : CRobo.minPos);
            changedINT = true;
        } else if (!gamepad2.a) {
            changedINT = false;
        }
    }

    private void ToggleOuttake() {
        if (gamepad1.a && !changedOUT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce
            // este dupa :
            robot.servoGhiaraOut.setPosition(robot.servoGhiaraOut.getPosition() == 0 ? CRobo.maxPos : CRobo.minPos);
            changedOUT = true;
        } else if (!gamepad1.a) {
            changedOUT = false;
        }
    }

    private void AutoPrindere() {
        double val = robot.dSensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", val);

        if (val < 3.7) {
            robot.servoGhiaraInt.setPosition(0);
            arePiesa = true;
            if (gamepad2.a) {
                robot.servoGhiaraInt.setPosition(CRobo.maxPosINT);
            }
        }
    }

    private void IntakeAxonMotion(){
        boolean axonOn = false;
        boolean move = true;
        if (gamepad2.x && move) {
            robot.intakeAxonLeft.setPosition(axonOn ? 0 : 0.25);
            robot.intakeAxonRight.setPosition(axonOn ? 0 : 0.25);
            axonOn = !axonOn;
            move = false;
        }else if(!gamepad2.x){
            move = true;
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

        telemetry.update();
    }

}