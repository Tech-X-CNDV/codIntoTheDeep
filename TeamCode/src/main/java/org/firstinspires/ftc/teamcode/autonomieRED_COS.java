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

    int xPos = -57;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-12, 61, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        OuttakeSliders outtakeSliders = new OuttakeSliders(hardwareMap);
        IntakeSliders intakeSliders = new IntakeSliders(hardwareMap);
        OuttakeClaw outtakeClaw = new OuttakeClaw(hardwareMap);
        IntakeClaw intakeClaw = new IntakeClaw(hardwareMap);
        OuttakeAxon outtakeAxon = new OuttakeAxon(hardwareMap);
        IntakeAxon intakeAxon = new IntakeAxon(hardwareMap);

        TrajectoryActionBuilder StartToSub = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-6.8,34));

        TrajectoryActionBuilder PrepareForMoveToHPlayer = StartToSub.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-35, 40.3, Math.toRadians(-90)), Math.toRadians(180))
                .strafeTo(new Vector2d(-37,11))
                .strafeTo(new Vector2d(-48,11));

        TrajectoryActionBuilder MovePartsToHPlayer = PrepareForMoveToHPlayer.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(11)
                .strafeTo(new Vector2d(-57,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .lineToY(11)
                .strafeTo(new Vector2d(-63,11))
                .setTangent(Math.toRadians(90))
                .lineToY(50);

        TrajectoryActionBuilder PrepareForSub = MovePartsToHPlayer.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .lineToY(50)
                .splineToSplineHeading(new Pose2d(-47.1,59, Math.toRadians(-90)), Math.toRadians(130));
        
        TrajectoryActionBuilder GoToSub = PrepareForSub.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-6.8,34, Math.toRadians(89.9)), Math.toRadians(350));

        TrajectoryActionBuilder GoToHPlayer = GoToSub.endTrajectory().fresh()
                .splineToSplineHeading(new Pose2d(-47.1,59, Math.toRadians(-90)), Math.toRadians(130));

        Action trajectoryActionCloseOut = MovePartsToHPlayer.endTrajectory().fresh()
                .strafeTo(new Vector2d(-36, -56))
                .build();

        TrajectoryActionBuilder Test = StartToSub.endTrajectory().fresh()
                .turn(90);

        int visionOutputPosition = 1; // dummy value (temporary)
        drive.updatePoseEstimate();
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        outtakeSliders.slidersDown(),
//                        intakeSliders.slidersDown(),
//                        outtakeClaw.closeClaw(),
//                        intakeClaw.closeClaw(),
//                        outtakeAxon.axonDown(),
//                        intakeAxon.axonUp()
//                )
//        );

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

//        Actions.runBlocking(
//                new SequentialAction(
//                        StartToSub.build(),
//                        Test.build()
//                )
//        );
        Actions.runBlocking(
                new SequentialAction(
                        StartToSub.build(),
                        PrepareForMoveToHPlayer.build(),
                        MovePartsToHPlayer.build(),
                        PrepareForSub.build(),
                        GoToSub.build(),
                        GoToHPlayer.build()
                )
        );
        sleep(100000);

//        Actions.runBlocking(
//                new SequentialAction(
//                        StartToSub.build(),
//                        // TODO add the outtake and claw actions
//                        PrepareForMoveToHPlayer.build()
//                )
//        );
//        for (int i = 0; i < 2; i++) {
//            Actions.runBlocking(
//                    MovePartsToHPlayer.build()
//            );
//            xPos -= 5;
//        }
//        Actions.runBlocking(
//                // TODO add the outtake and claw actions
//                PrepareForSub.build()
//        );
//        for (int i = 0; i < 3; i++) {
//            Actions.runBlocking(
//                    new SequentialAction(
//                            GoToSub.build(),
//                            // TODO Add the outtake and claw actions
//                            GoToHPlayer.build()
//                    )
//            );
//        }
    }
}