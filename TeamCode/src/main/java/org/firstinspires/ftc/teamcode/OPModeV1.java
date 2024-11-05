
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

class CRobo {

    // Servouri
    public Servo servoGhiaraOut;
    public Servo servoGhiaraInt;
    public Servo servoRotireInt;
    // Senzori
    public DistanceSensor dSensor;
    // Variabile
    static final double maxPos = 0.19;
    static final double maxPosINT = 0.5;
    static final double minPos = 0.0;
    static final double maxPosROTINT = 0.5;
    // Motoare
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;

    public void init() {
        // Initializare servo si senzor
        servoGhiaraOut = hardwareMap.get(Servo.class, "GHIARAOUTTAKE");
        servoGhiaraInt = hardwareMap.get(Servo.class, "GHIARAINTAKE");
        servoRotireInt = hardwareMap.get(Servo.class, "ROTIREGHIARAINTAKE");
        servoRotireInt.setPosition(minPos);
        servoGhiaraInt.setPosition(minPos);
        servoGhiaraOut.setPosition(minPos);
        dSensor = hardwareMap.get(DistanceSensor.class, "DISTANCESENSOR");
        // Initializare roti
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRearMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}

@TeleOp(name = "Basic: OPModeV1", group = "Linear OpMode")
public class OPModeV1 extends OpMode {
    public ElapsedTime runtime = new ElapsedTime();

    CRobo robot = new CRobo();

    @Override
    public void init() {

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
        ToggleRotireIntake();
        ToggleIntake();
        ToggleOuttake();
        AutoPrindere();
        Roti();
        Telemetry();
    }

    private void ToggleRotireIntake() {
        if (gamepad2.b && !changedROTINT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce este dupa :
            robot.servoRotireInt.setPosition(robot.servoRotireInt.getPosition() == 0 ? CRobo.maxPosROTINT : CRobo.minPos);
            changedROTINT = true;
        } else if (!gamepad2.a) {
            changedROTINT = false;
        }
    }

    private void ToggleIntake() {
        if (gamepad2.a && !changedINT) {
            arePiesa = false;
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce este dupa :
            robot.servoGhiaraInt.setPosition(robot.servoGhiaraInt.getPosition() == 0 ? CRobo.maxPosINT : CRobo.minPos);
            changedINT = true;
        } else if (!gamepad2.a) {
            changedINT = false;
        }
    }

    private void ToggleOuttake() {
        if (gamepad1.a && !changedOUT) {
            // if-else compact: daca este true pune ce este inainte de : daca este fals ce este dupa :
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

    private void Roti() {
        if (gamepad2.left_bumper && pressLbumper2) {
            speedLimit = !speedLimit;
            pressLbumper2 = false;
        }
        if (!gamepad2.left_bumper) {
            pressLbumper2 = true;
        }
        double r = Math.hypot(gamepad2.left_stick_x, -gamepad2.left_stick_y);
        double robotAngle = Math.atan2(-gamepad2.left_stick_y, gamepad2.left_stick_x) - Math.PI / 4;
        double rightX = -gamepad2.right_stick_x;

        double v1 = r * Math.cos(robotAngle) + rightX;
        double v2 = r * Math.sin(robotAngle) - rightX;
        double v3 = r * Math.sin(robotAngle) + rightX;
        double v4 = r * Math.cos(robotAngle) - rightX;

        if (speedLimit) {
            v1 /= 2.0d;
            v2 /= 2.0d;
            v3 /= 2.0d;
            v4 /= 2.0d;
        }
        v1 = v1 - v1 * 0.75d * this.gamepad2.right_trigger;
        v2 = v2 - v2 * 0.75d * this.gamepad2.right_trigger;
        v3 = v3 - v3 * 0.75d * this.gamepad2.right_trigger;
        v4 = v4 - v4 * 0.75d * this.gamepad2.right_trigger;
        robot.leftFrontMotor.setPower(v1);
        robot.rightFrontMotor.setPower(v2);
        robot.leftRearMotor.setPower(v3);
        robot.rightRearMotor.setPower(v4);
    }

    private void Telemetry() {
        double ghiaraintPos = robot.servoGhiaraInt.getPosition();
        double ghiaraintangle = ghiaraintPos * 180;
        double ghiaraoutPos = robot.servoGhiaraOut.getPosition();
        double ghiaraoutangle = ghiaraoutPos * 180;
        double servortireintPos = robot.servoRotireInt.getPosition();
        double servorotireangle = servortireintPos * 180;
        telemetry.addData("Status", "Initialized");

        telemetry.addData("Outtake Position", ghiaraoutPos);
        telemetry.addData("Outtake Angle", ghiaraoutangle);
        telemetry.addData("Intake Position", ghiaraintPos);
        telemetry.addData("Intake Angle", ghiaraintangle);
        telemetry.addData("Intake Position", servortireintPos);
        telemetry.addData("Intake Rotation Angle", servorotireangle);

        telemetry.update();
    }

}