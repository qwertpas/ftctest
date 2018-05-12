package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by leshaseletskiy on 4/21/18.
 */

public class Motors {
    private DcMotor aDrive;
    private DcMotor bDrive;
    private DcMotor cDrive;
    private DcMotor dDrive;

    public Motors(DcMotor aDrive, DcMotor bDrive, DcMotor cDrive, DcMotor dDrive) {
        this.aDrive = aDrive;
        this.bDrive = bDrive;
        this.cDrive = cDrive;
        this.dDrive = dDrive;
    }

    public void moveGlobalAngle(double angle, double heading, double power){
        //move the robot in an angle relative to the field
        double radians = Math.toRadians(angle - 45);

        double[] cartesianCoords = Calculate.polarToCartesian(power, radians,true);
        double[] globalVector = Calculate.FOD(cartesianCoords[0], cartesianCoords[1], heading,false,true);

        double x = globalVector[0];
        double y = globalVector[1];
        aDrive.setPower(-x);
        bDrive.setPower(y);
        cDrive.setPower(x);
        dDrive.setPower(-y);
    }

    public void moveLocalAngle(double angle, double power){
        //move the robot in an angle relative to the robot's heading
        double degrees = Math.toRadians(angle - 45);
        double x = Math.cos(degrees) * power;
        double y = Math.sin(degrees) * power;
        aDrive.setPower(-x);
        bDrive.setPower(y);
        cDrive.setPower(x);
        dDrive.setPower(-y);
    }

    public void forward(double forward) {
        aDrive.setPower(-forward);
        bDrive.setPower(forward);
        cDrive.setPower(forward);
        dDrive.setPower(-forward);
    }

    public void right(double right) {
        aDrive.setPower(right);
        bDrive.setPower(right);
        cDrive.setPower(-right);
        dDrive.setPower(-right);
    }

    public void forwardleft(double amount) {
        aDrive.setPower(-amount);
        bDrive.setPower(0);
        cDrive.setPower(amount);
        dDrive.setPower(0);
    }

    public void forwardright(double amount) {
        aDrive.setPower(0);
        bDrive.setPower(amount);
        cDrive.setPower(0);
        dDrive.setPower(-amount);
    }

    public void clearmotors() {
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
    }

    public void aMotor() {
        aDrive.setPower(1);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
    }

    public void bMotor() {
        aDrive.setPower(0);
        bDrive.setPower(1);
        cDrive.setPower(0);
        dDrive.setPower(0);
    }

    public void cMotor() {
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(1);
        dDrive.setPower(0);
    }

    public void dMotor() {
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(1);
    }
}
