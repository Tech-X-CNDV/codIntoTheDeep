package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class axontest2 extends OpMode {
    public Servo servo2;

    @Override
    public void init() {
        servo2 = hardwareMap.get(Servo.class, "ROTIREGHIARAINTAKE");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo2.setPosition(0);
        }
        if (gamepad1.b) {
            servo2.setPosition(1);
        }
    }
}