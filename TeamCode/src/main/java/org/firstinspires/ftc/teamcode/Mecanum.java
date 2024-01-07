package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Mecanum1", group="Iterative Opmode")
public class Mecanum extends OpMode {

    // declare and initialize four DcMotors.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_left   = null;
    private DcMotor back_right  = null;
    private DcMotor elevL  = null;
    private DcMotor elevR  = null;
    private DcMotor rotatorL = null;
    private DcMotor rotatorR = null;

    Servo   servo1;
    Servo   servo0;
    Servo wrist;
    Servo grabber;

    double wristPosition = 0;
    int elevatorZero=0;
    @Override
    public void init() {
        // Name strings must match up with the config on the Robot Controller
        // app.
        front_left   = hardwareMap.get(DcMotor.class, "frontLeft");
        front_right  = hardwareMap.get(DcMotor.class, "frontRight");
        back_left    = hardwareMap.get(DcMotor.class, "backLeft");
        back_right   = hardwareMap.get(DcMotor.class, "backRight");

        elevL = hardwareMap.get(DcMotor.class,"motor0");
        elevR = hardwareMap.get(DcMotor.class,"motor1");

        rotatorL = hardwareMap.get(DcMotor.class,"motor2");
        rotatorR = hardwareMap.get(DcMotor.class,"motor3");

        wrist = hardwareMap.get(Servo.class, "Servo0");
        servo1 = hardwareMap.get(Servo.class, "Servo1");

        front_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);

        rotatorL.setDirection(DcMotor.Direction.FORWARD);
        rotatorR.setDirection(DcMotor.Direction.REVERSE);

        elevL.setDirection(DcMotor.Direction.FORWARD);
        elevR.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double drive;
        double strafe;
        double twist;

        if(gamepad1.right_bumper){
            strafe= gamepad1.right_stick_x;
            drive = gamepad1.left_stick_y;
            twist  = gamepad1.left_stick_x;
        }
        else{
            strafe = gamepad1.right_stick_x*0.5;
            drive = gamepad1.left_stick_y*0.5;
            twist  = gamepad1.left_stick_x*0.5;
        }

        //Elbow - motor xy
        if(gamepad1.y){
            elevL.setPower(0.3);
            elevR.setPower(0.3);
        }

        else if(gamepad1.x){
            elevL.setPower(-0.3);
            elevR.setPower(-0.3);
        }

        else{
            elevL.setPower(0);
            elevR.setPower(0);
        }

        //Rotator - motor
        if(gamepad1.dpad_up){
            rotatorL.setPower(0.2);
            rotatorR.setPower(0.2);
        } else if (gamepad1.dpad_down) {
            rotatorL.setPower(-0.2);
            rotatorR.setPower(-0.2);
        } else{
            rotatorL.setPower(0);
            rotatorR.setPower(0);
        }

        //Grabber
        if(gamepad1.a){
            grabber.setPosition(0.2);
        }

        if(gamepad1.b){
            grabber.setPosition(0);
        }

        if(gamepad1.right_trigger>0.01){
            wristPosition= wristPosition+0.01;
            wrist.setPosition(wristPosition);
        }
        else if(gamepad1.left_trigger>0.01){
            wristPosition= wristPosition-0.01;
            wrist.setPosition(wristPosition);
        }
        else{
            wrist.setPosition(wristPosition);
        }

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive + strafe +twist),
                (drive - strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        front_left.setPower(speeds[0]);
        front_right.setPower(speeds[1]);
        back_left.setPower(speeds[2]);
        back_right.setPower(speeds[3]);

        telemetry.addData("Rotator L Pos",rotatorL.getCurrentPosition());
        telemetry.update();
    }
}

