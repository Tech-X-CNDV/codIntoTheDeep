
package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
        if (robot.dSensor != null) {
            AutoPrindere();
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
        } else if (!gamepad2.a) {
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
        double ghiaraintPos = robot.servoGhiaraInt.getPosition();
        double ghiaraintangle = ghiaraintPos * 180;
        double ghiaraoutPos = robot.servoGhiaraOut.getPosition();
        double ghiaraoutangle = ghiaraoutPos * 180;
        double servortireintPos = robot.servoRotireInt.getPosition();
        double servorotireangle = servortireintPos * 180;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Outtake Position", ghiaraoutPos);
        telemetry.addData("Outtake Angle", ghiaraoutangle);
        telemetry.addData("Intake Position", ghiaraintPos);
        telemetry.addData("Intake Angle", ghiaraintangle);
        telemetry.addData("Intake Position", servortireintPos);
        telemetry.addData("Intake Rotation Angle", servorotireangle);

        telemetry.update();
    }

}