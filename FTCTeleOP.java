package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FTCTeleOP extends OpMode {
    DcMotor FLM, FRM, BLM, BRM, clawM;
    Servo clawS;
    double xaxLisL, yaxisL;
    double TOLERANCE = 0.3;
    BNO055IMU imu;
    double LF = 0;
    double RF = 0;
    double LB = 0;
    double RB = 0;
    double vertical;
    ElapsedTime SCooldown = new ElapsedTime();
    double COOLDOWN = 0.3;

    @Override
    public void init() {
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");

        clawM = hardwareMap.dcMotor.get("clawM");

        clawS = hardwareMap.servo.get("clawS");

        BRM.setDirection(DcMotor.Direction.REVERSE);
        FRM.setDirection(DcMotor.Direction.REVERSE);
        imu = (BNO055IMU)hardwareMap.get("imu");
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;
        double rol = -angles.secondAngle;
        double pitch = -angles.thirdAngle;
        FLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SCooldown.reset();
        update();

    }


    @Override
    public void loop() {
        vertical = -gamepad2.left_stick_y;
        motorDrive();
        clawM.setPower(vertical/2);

        // activate claw
        if(gamepad2.a && SCooldown.seconds() >= COOLDOWN){
            SCooldown.reset();
            clawS.setPosition(0.8);
        } else if(gamepad2.b && SCooldown.seconds() >= COOLDOWN){
            SCooldown.reset();
            clawS.setPosition(0);
        }
        update();

    }

    //telemetry
    public void update(){
        telemetry.addData("Directions: ", "b to grab, a to release");
        telemetry.addData("FLM Power: ", FLM.getPower());
        telemetry.addData("FRM Power: ", FRM.getPower());
        telemetry.addData("BLM Power: ", BLM.getPower());
        telemetry.addData("BRM Power: ", BRM.getPower());
        telemetry.update();
    }

    // speed boost, if bumpers = 0 set speed 1/2
    public void motorDrive() {
        DirAxes();
        if(gamepad1.left_bumper || gamepad1.right_bumper){
            FRM.setPower(RF);
            FLM.setPower(LF);
            BRM.setPower(RB);
            BLM.setPower(LB);
        } else{
            FRM.setPower(RF/2);
            FLM.setPower(LF/2);
            BRM.setPower(RB/2);
            BLM.setPower(LB/2);
        }
    }

    // direction control left
    public void DirAxes(){
        double yaxL = -gamepad1.left_stick_y;
        double xaxL = gamepad1.left_stick_x;
        LF = 0;
        RF = 0;
        LB = 0;
        RB = 0;

        LF += yaxL;
        RF += yaxL;
        LB += yaxL;
        RB += yaxL;

        // strafe left/right
        LF += xaxL;
        RF -= xaxL;
        LB -= xaxL;
        RB += xaxL;

        // turning left or right
        double yaxR = -gamepad1.right_stick_y;
        double xaxR = gamepad1.right_stick_x;


        LF += xaxR;
        RF -= xaxR;
        LB += xaxR;
        RB -= xaxR;

        // Caps the speed to -1 and 1
        if(LF > 1 || LF < -1){
            if(LF > 1){
                LF = 1;
            }else{
                LF = -1;
            }
        }
        if(RF > 1 || RF < -1){
            if(RF > 1){
                RF = 1;
            }else{
                RF = -1;
            }
        }
        if(LB > 1 || LF < -1){
            if(LB > 1){
                LB = 1;
            }else{
                LB = -1;
            }
        }
        if(RB > 1 || RB < -1){
            if(RB > 1){
                RB = 1;
            }else{
                RB = -1;
            }
        }
        

    }
}
