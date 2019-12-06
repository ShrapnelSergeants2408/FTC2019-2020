package org.firstinspires.ftc.teamcode;
import java.lang.Math;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class Autonomous1 extends LinearOpMode {
    double yaw,rol,pitch;
    BNO055IMU imu;
    DcMotor FLM,FRM,BLM,BRM;
    double TicksPerRev = 537.6;
    double WheelDiameterInches = 4;
    int TicksPerInch = (int)(TicksPerRev / (Math.PI*WheelDiameterInches));
    @Override
    public void runOpMode(){
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FRM.setDirection(DcMotor.Direction.REVERSE);
        BRM.setDirection(DcMotor.Direction.REVERSE);

        FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = (BNO055IMU)hardwareMap.get("imu");
        initGyro();
        getAngles();

        waitForStart();
        if(opModeIsActive()){
            getAngles();
            Drive(0.5,24);
            StrafeRight(0.5,28);

        }
        //Drive(0.5,24);

        //StrafeRight(0.5, 36);

        //drop arm to front and grab build platform

        //Reverse(0.5, 24);

        //StrafeLeft(0.5, 24);

        //Drive(0.5, 48);

        //Turn(0.25, 180);

        //Drive(0.5, 24);

    }

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void getAngles(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yaw = -angles.firstAngle;
        rol = -angles.secondAngle;
        pitch = -angles.thirdAngle;
    }

    public void Drive(double speed, double inches){
        if(opModeIsActive()) {
            FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int LTarget = (int)( (FLM.getCurrentPosition()+BLM.getCurrentPosition())/2 + (TicksPerInch * inches));
            int RTarget = (int)( (FRM.getCurrentPosition()+BRM.getCurrentPosition())/2 + (TicksPerInch * inches));
            FLM.setTargetPosition(LTarget);
            FRM.setTargetPosition(RTarget);
            BLM.setTargetPosition(LTarget);
            BRM.setTargetPosition(RTarget);
            FLM.setPower(speed);
            FRM.setPower(speed);
            BLM.setPower(speed);
            BRM.setPower(speed);

            FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FLM.isBusy() && opModeIsActive() || FRM.isBusy() && opModeIsActive() || BLM.isBusy() && opModeIsActive() || BRM.isBusy() && opModeIsActive()) {
                int LeftPosition = (FLM.getCurrentPosition() + BLM.getCurrentPosition())/2;
                int RightPosition = (FRM.getCurrentPosition() + BRM.getCurrentPosition())/2;
                int LeftTarget = (FLM.getTargetPosition() + BLM.getTargetPosition())/2;
                int RightTarget = (FRM.getTargetPosition() + BRM.getTargetPosition())/2;
                telemetry.addData("Left Position", (LeftPosition * TicksPerInch));
                telemetry.addData("Right Position", (RightPosition * TicksPerInch));
                telemetry.addData("Left Target: ", LeftTarget-(LeftPosition*TicksPerInch));
                telemetry.addData("Right Target: ", RightTarget-(LeftPosition*TicksPerInch));
                telemetry.update();
            }
            FLM.setPower(0);
            FRM.setPower(0);
            BLM.setPower(0);
            BRM.setPower(0);

            FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void Reverse(double speed, double inches){
        if(opModeIsActive()) {
            FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int LTarget = (int)( (FLM.getCurrentPosition()+BLM.getCurrentPosition())/2 - (TicksPerInch * inches));
            int RTarget = (int)( (FRM.getCurrentPosition()+BRM.getCurrentPosition())/2 - (TicksPerInch * inches));
            FLM.setTargetPosition(LTarget);
            FRM.setTargetPosition(RTarget);
            BLM.setTargetPosition(LTarget);
            BRM.setTargetPosition(RTarget);
            FLM.setPower(-speed);
            FRM.setPower(-speed);
            BLM.setPower(-speed);
            BRM.setPower(-speed);

            FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FLM.isBusy() && opModeIsActive() || FRM.isBusy() && opModeIsActive() || BLM.isBusy() && opModeIsActive() || BRM.isBusy() && opModeIsActive()) {
                int LeftPosition = (FLM.getCurrentPosition() + BLM.getCurrentPosition())/2;
                int RightPosition = (FRM.getCurrentPosition() + BRM.getCurrentPosition())/2;
                int LeftTarget = (FLM.getTargetPosition() + BLM.getTargetPosition())/2;
                int RightTarget = (FRM.getTargetPosition() + BRM.getTargetPosition())/2;
                telemetry.addData("Left Position", (LeftPosition * TicksPerInch));
                telemetry.addData("Right Position", (RightPosition * TicksPerInch));
                telemetry.addData("Left Target: ", LeftTarget-(LeftPosition*TicksPerInch));
                telemetry.addData("Right Target: ", RightTarget-(LeftPosition*TicksPerInch));
                telemetry.update();
            }
            FLM.setPower(0);
            FRM.setPower(0);
            BLM.setPower(0);
            BRM.setPower(0);

            FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void StrafeLeft(double speed, double inches){
        if(opModeIsActive()) {
            FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int FLTarget = (int)( (FLM.getCurrentPosition()+BLM.getCurrentPosition())/2 - (TicksPerInch * inches));
            int FRTarget = (int)( (FRM.getCurrentPosition()+BRM.getCurrentPosition())/2 + (TicksPerInch * inches));
            int BLTarget = (int)( (FLM.getCurrentPosition()+BLM.getCurrentPosition())/2 + (TicksPerInch * inches));
            int BRTarget = (int)( (FRM.getCurrentPosition()+BRM.getCurrentPosition())/2 - (TicksPerInch * inches));
            FLM.setTargetPosition(FLTarget);
            FRM.setTargetPosition(FRTarget);
            BLM.setTargetPosition(BLTarget);
            BRM.setTargetPosition(BRTarget);
            FLM.setPower(-speed);
            FRM.setPower(speed);
            BLM.setPower(speed);
            BRM.setPower(-speed);

            FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FLM.isBusy() && opModeIsActive() || FRM.isBusy() && opModeIsActive() || BLM.isBusy() && opModeIsActive() || BRM.isBusy() && opModeIsActive()) {
                int LeftPosition = (FLM.getCurrentPosition() + BLM.getCurrentPosition())/2;
                int RightPosition = (FRM.getCurrentPosition() + BRM.getCurrentPosition())/2;
                int LeftTarget = (FLM.getTargetPosition() + BLM.getTargetPosition())/2;
                int RightTarget = (FRM.getTargetPosition() + BRM.getTargetPosition())/2;
                telemetry.addData("Left Position", (LeftPosition * TicksPerInch));
                telemetry.addData("Right Position", (RightPosition * TicksPerInch));
                telemetry.addData("Left Target: ", LeftTarget-(LeftPosition*TicksPerInch));
                telemetry.addData("Right Target: ", RightTarget-(LeftPosition*TicksPerInch));
                telemetry.update();
            }
            FLM.setPower(0);
            FRM.setPower(0);
            BLM.setPower(0);
            BRM.setPower(0);

            FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void StrafeRight(double speed, double inches){
        if(opModeIsActive()) {
            FLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            int FLTarget = (int)( (FLM.getCurrentPosition()+BLM.getCurrentPosition())/2 + (TicksPerInch * inches));
            int FRTarget = (int)( (FRM.getCurrentPosition()+BRM.getCurrentPosition())/2 - (TicksPerInch * inches));
            int BLTarget = (int)( (FLM.getCurrentPosition()+BLM.getCurrentPosition())/2 - (TicksPerInch * inches));
            int BRTarget = (int)( (FRM.getCurrentPosition()+BRM.getCurrentPosition())/2 + (TicksPerInch * inches));
            FLM.setTargetPosition(FLTarget);
            FRM.setTargetPosition(FRTarget);
            BLM.setTargetPosition(BLTarget);
            BRM.setTargetPosition(BRTarget);
            FLM.setPower(speed);
            FRM.setPower(-speed);
            BLM.setPower(-speed);
            BRM.setPower(speed);

            FLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FLM.isBusy() && opModeIsActive() || FRM.isBusy() && opModeIsActive() || BLM.isBusy() && opModeIsActive() || BRM.isBusy() && opModeIsActive()) {
                int LeftPosition = (FLM.getCurrentPosition() + BLM.getCurrentPosition())/2;
                int RightPosition = (FRM.getCurrentPosition() + BRM.getCurrentPosition())/2;
                int LeftTarget = (FLM.getTargetPosition() + BLM.getTargetPosition())/2;
                int RightTarget = (FRM.getTargetPosition() + BRM.getTargetPosition())/2;
                telemetry.addData("Left Position", (LeftPosition * TicksPerInch));
                telemetry.addData("Right Position", (RightPosition * TicksPerInch));
                telemetry.addData("Left Target: ", LeftTarget-(LeftPosition*TicksPerInch));
                telemetry.addData("Right Target: ", RightTarget-(LeftPosition*TicksPerInch));
                telemetry.update();
            }
            FLM.setPower(0);
            FRM.setPower(0);
            BLM.setPower(0);
            BRM.setPower(0);

            FLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void Turn(double speed, double degree){
        getAngles();
        double zeroDegree = yaw;
        if(zeroDegree < 0){
            zeroDegree += 360;
        }
        double targetDegree = degree + zeroDegree;
        double DEGREETOLERANCE = 3;

        //Loop until degree met
        while(zeroDegree <= targetDegree - DEGREETOLERANCE && opModeIsActive() || zeroDegree >= targetDegree + DEGREETOLERANCE && opModeIsActive()){
            getAngles();
            zeroDegree = yaw;
            telemetry.addData("Yaw: ", yaw);
            telemetry.addData("Target Degree: ", degree);
            if(zeroDegree < 0){
                zeroDegree += 360;
            }
            if(degree > 180){
                // Turn Left
                telemetry.addData("Turning: ", "Left");
                FLM.setPower(-speed);
                FRM.setPower(speed);
                BLM.setPower(-speed);
                BRM.setPower(speed);
            } else if (degree <= 180){
                // Turn Right
                telemetry.addData("Turning: ", "Right");
                FLM.setPower(speed);
                FRM.setPower(-speed);
                BLM.setPower(speed);
                BRM.setPower(-speed);
            }
            telemetry.addData("FLM Speed: ", FLM.getPower());
            telemetry.addData("FRM Speed: ", FRM.getPower());
            telemetry.addData("BLM Speed: ", BLM.getPower());
            telemetry.addData("BRM Speed: ", BRM.getPower());
            telemetry.update();
        }
        FLM.setPower(0);
        FRM.setPower(0);
        BLM.setPower(0);
        BRM.setPower(0);

    }
}
