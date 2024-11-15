//package org.firstinspires.ftc.teamcode;
//
//import android.graphics.Color;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.Servo;
//
//
//@TeleOp(name = "Basic: axontest", group = "Linear OpMode")
//public class axontest extends OpMode {
//    Servo intakeAxonright,intakeAxonleft;
//    Servo outtakeAxonright,outtakeAxonleft;
//    boolean changedaxonint;
//    // values is a reference to the hsvValues array.
//
//    @Override
//    public void init() {
//         servo = hardwarewMap.get(Servo.class, "ROTIREGHIARAINTAKE");
//        //outtakeAxonright=hardwareMap.get(Servo.class, "OUTAXONRIGHT");
//        // outtakeAxonleft=hardwareMap.get(Servo.class, "OUTAXONLEFT");
//        // intakeAxonright=hardwareMap.get(Servo.class, "INTAKERIGHT");
//        // intakeAxonleft=hardwareMap.get(Servo.class,"INTAKELEFT");
//        //axoncontinuu=hardwareMap.get(CRServo.class,"AXONCONTINUU");
//
//        //intakeAxonleft.setPosition(0);
//        //intakeAxonright.setPosition(0);
//    }
//public double val=0.5;
//    @Override
//    public void loop() {
//
//
//        while(gamepad1.a){
//            servo.setPower(1);
//        }
//        while(gamepad1.b){
//            servo.setPower(-1);
//        }
//        servo.setpower(0);
//        /*
//        if(gamepad2.y){
//            outtakeAxonright.setPosition(1);
//            outtakeAxonleft.setPosition(1);
//        }
//        if(gamepad2.b) {
//            outtakeAxonright.setPosition(0);
//            outtakeAxonleft.setPosition(0);
//        }
//        */
//
//        //telemetry.addData("Putiere: ",)
//        /*
//        if(gamepad2.right_stick_y > 0)
//        {
//            if(val<1)val+=0.01;
//        }
//            else if(gamepad2.right_stick_y<0){
//                if(val>0)val-=0.01;
//        }
//            outtakeAxonright.setPosition(val);
//            outtakeAxonleft.setPosition(val);
//            telemetry.addData("Angle ",outtakeAxonright.getPosition()*255);
//            telemetry.addData("val ",val);
//
//
//         */
//
//        if(gamepad2.x){
//            intakeAxonleft.setPosition(0.21);
//            intakeAxonright.setPosition(0.21);
//
//        }
//        if(gamepad2.a){
//            intakeAxonleft.setPosition(0);
//            intakeAxonright.setPosition(0);
//        }
//
//
//    }
//}
