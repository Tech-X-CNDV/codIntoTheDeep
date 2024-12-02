package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(name = "autoRED_COS")
public class autonomieRED_COS extends LinearOpMode{

    public class OuttakeSliders {
        private DcMotor outtakeSliderUp;
        private DcMotor outtakeSliderDown;

        public OuttakeSliders(HardwareMap hardwareMap) {
            outtakeSliderUp = hardwareMap.get(DcMotor.class, "outtakeSliderUp");
            outtakeSliderDown = hardwareMap.get(DcMotor.class, "outtakeSliderDown");
            outtakeSliderUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSliderDown.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeSliderUp.setDirection(DcMotorSimple.Direction.FORWARD);
            outtakeSliderDown.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class SlidersUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeSliderUp.setPower(1);
                    outtakeSliderDown.setPower(1);
                    initialized = true;
                }
                // checks lift's current position
                double pos = outtakeSliderUp.getCurrentPosition();
                packet.put("outtakePos", pos);
                if (pos < 4150.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    outtakeSliderUp.setPower(0);
                    outtakeSliderDown.setPower(0);
                    return false;
                }
            }
        }
        public Action slidersUp() {
            return new SlidersUp();
        }

        public class SlidersDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeSliderUp.setPower(-1);
                    outtakeSliderDown.setPower(-1);
                    initialized = true;
                }

                double pos = outtakeSliderUp.getCurrentPosition();
                packet.put("outtakePos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    outtakeSliderUp.setPower(0);
                    outtakeSliderDown.setPower(0);
                    return false;
                }
            }
        }
        public Action slidersDown(){
            return new SlidersDown();
        }
    }

    public class OuttakeAxon {
        private Servo outtakeAxonLeft;
        private Servo outtakeAxonRight;

        public OuttakeAxon(HardwareMap hardwareMap){
            outtakeAxonLeft = hardwareMap.get(Servo.class, "outtakeAxonLeft");
            outtakeAxonRight = hardwareMap.get(Servo.class, "outtakeAxonRight");
        }

        public class AxonUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeAxonLeft.setPosition(1);
                outtakeAxonRight.setPosition(1);
                return false;
            }
        }

        public Action axonUp(){
            return new AxonUp();
        }

        public class AxonDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                outtakeAxonLeft.setPosition(0.4);
                outtakeAxonRight.setPosition(0.4);
                return false;
            }
        }

        public Action axonDown(){
            return new AxonDown();
        }
    }

    public class OuttakeClaw {
        private Servo claw;

        public OuttakeClaw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "ghiaraOuttake");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.3);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    public class IntakeSliders {
        private DcMotor intakeSlider;

        public IntakeSliders(HardwareMap hardwareMap) {
            intakeSlider = hardwareMap.get(DcMotor.class, "intakeSlider");
            intakeSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class SlidersUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeSlider.setPower(1);
                    initialized = true;
                }
                // checks lift's current position
                double pos = intakeSlider.getCurrentPosition();
                packet.put("intakePos", pos);
                if (pos < 680.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    intakeSlider.setPower(0);
                    return false;
                }
            }
        }
        public Action slidersUp() {
            return new SlidersUp();
        }

        public class SlidersDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeSlider.setPower(-1);
                    initialized = true;
                }

                double pos = intakeSlider.getCurrentPosition();
                packet.put("intakePos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    intakeSlider.setPower(0);
                    return false;
                }
            }
        }
        public Action slidersDown(){
            return new SlidersDown();
        }
    }

    public class IntakeAxon {
        private Servo intakeAxonLeft;
        private Servo intakeAxonRight;

        public IntakeAxon(HardwareMap hardwareMap){
            intakeAxonLeft = hardwareMap.get(Servo.class, "intakeAxonLeft");
            intakeAxonRight = hardwareMap.get(Servo.class, "intakeAxonRight");
        }

        public class AxonUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeAxonLeft.setPosition(0);
                intakeAxonRight.setPosition(0);
                return false;
            }
        }

        public Action axonUp(){
            return new AxonUp();
        }

        public class AxonDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeAxonLeft.setPosition(0.27);
                intakeAxonRight.setPosition(0.27);
                return false;
            }
        }

        public Action axonDown(){
            return new AxonDown();
        }
    }

    public class IntakeClaw {
        private Servo claw;

        public IntakeClaw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "ghiaraIntake");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.3);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
        Pose2d initialPose = new Pose2d(-36, -56, Math.toRadians(90));
        OuttakeSliders outtakeSliders = new OuttakeSliders(hardwareMap);
        IntakeSliders intakeSliders = new IntakeSliders(hardwareMap);
        OuttakeClaw outtakeClaw = new OuttakeClaw(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);
        OuttakeAxon outtakeAxon = new OuttakeAxon(hardwareMap);
        IntakeAxon intakeAxon = new IntakeAxon(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .turn(Math.toRadians(-45))
                .strafeTo(new Vector2d(-46.7,-45.7));

        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-47.3,-40.2));

        Action trajectoryActionCloseOut = tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-36, -56))
                .build();

        int visionOutputPosition = 1; // dummy value (temporary)

        Actions.runBlocking(
                new SequentialAction(
                        outtakeSliders.slidersDown(),
                        intakeSliders.slidersDown(),
                        outtakeClaw.closeClaw(),
                        intakeClaw.closeClaw(),
                        outtakeAxon.axonDown(),
                        intakeAxon.axonUp()
                )
        );

        while (!isStopRequested() && !opModeIsActive()) { // these lines until waitforstart() can be ignored.
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }
        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        outtakeSliders.slidersUp(),
                        outtakeClaw.openClaw(),
                        outtakeSliders.slidersDown(),
                        tab2.build(),
                        trajectoryActionCloseOut
                )
        );
    }
}