package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;


//this is a bit different from regular Motors class because you have to call execute to actually send the power to the motors
//This is more convenient when you want the drivetrain to do multiple things at once such as move in circle while spinning
//also there is a getter for every motor power
public class Motors2 {
    private DcMotor aDrive;
    private DcMotor bDrive;
    private DcMotor cDrive;
    private DcMotor dDrive;

    private double aPower = 0;
    private double bPower = 0;
    private double cPower = 0;
    private double dPower = 0;

    public Motors2(DcMotor aDrive, DcMotor bDrive, DcMotor cDrive, DcMotor dDrive) {
        this.aDrive = aDrive;
        this.bDrive = bDrive;
        this.cDrive = cDrive;
        this.dDrive = dDrive;
    }

    //call this every loop to make sure the power sent to the motors is updated
    //also does a last clip to make sure power never exceeds 1
    public void execute(){
        aDrive.setPower(Range.clip(aPower,-1,1));
        bDrive.setPower(Range.clip(bPower,-1,1));
        cDrive.setPower(Range.clip(cPower,-1,1));
        dDrive.setPower(Range.clip(dPower,-1,1));
    }


        //THE GETTERS ##################################################

    public double[] getPowers(){ return new double[] {aPower, bPower, cPower, dPower}; }
    public double getAPower(){ return aPower; }
    public double getBPower(){ return bPower; }
    public double getCPower(){ return cPower; }
    public double getDPower(){ return dPower; }


        //THE SETTERS ##################################################

    //move the robot in an angle relative to the field
    public void moveGlobalAngle(double angle, double heading, double power, double spin) {
        double radians = Math.toRadians(angle - 45);
        double[] cartesianCoords = Calculate.polarToCartesian(power, radians, true);
        double[] globalVector = Calculate.FOD(cartesianCoords[0], cartesianCoords[1], -heading, false, true);
        double x = globalVector[0];
        double y = globalVector[1];
        aPower = -x + spin;
        bPower = y + spin;
        cPower = x + spin;
        dPower = -y + spin;
    }

    //move the robot in an angle relative to the robot's heading
    public void moveLocalAngle(double angle, double power, double spin) {
        double degrees = Math.toRadians(angle - 45);
        double x = Math.cos(degrees) * power;
        double y = Math.sin(degrees) * power;
        aPower = -x + spin;
        bPower = y + spin;
        cPower = x + spin;
        dPower = -y + spin;
    }

    //zeros them out (stops drivetrain if execute() is run)
    public void clear() {
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
        execute();
    }

    //stops and closes motors entirely
    public void stop() {
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
        execute();
        aDrive.close();
        bDrive.close();
        cDrive.close();
        dDrive.close();
    }


    public void setPowers(double aPower, double bPower, double cPower, double dPower) {
        this.aPower = aPower;
        this.bPower = bPower;
        this.cPower = cPower;
        this.dPower = dPower;
    }
    public void setAPower(double power){ aPower = power; }
    public void setBPower(double power){ bPower = power; }
    public void setCPower(double power){ cPower = power; }
    public void setDPower(double power){ dPower = power; }

    //premade directions
    public void forward(double forward) {
        aPower = -forward;
        bPower = forward;
        cPower = forward;
        dPower = -forward;
    }
    public void right(double right) {
        aPower = right;
        bPower = right;
        cPower = -right;
        dPower = -right;
    }
    public void forwardleft(double amount) {
        aPower = -amount;
        bPower = 0;
        cPower = amount;
        dPower = 0;
    }
    public void forwardright(double amount) {
        aPower = 0;
        bPower = amount;
        cPower = 0;
        dPower = -amount;
    }


        //THE ADDERS ##################################################
        //MAKE SURE NOT TO RUN MULTIPLE TIMES OR IN A LOOP
    public void addPowers(double aPower, double bPower, double cPower, double dPower) {
        this.aPower = this.aPower + (aPower);
        this.bPower = this.bPower + (bPower);
        this.cPower = this.cPower + (cPower);
        this.dPower = this.dPower + (dPower);
    }
    public void addAPower(double add){ aPower = aPower + add; }
    public void addBPower(double add){ bPower = bPower + add; }
    public void addCPower(double add){ cPower = cPower + add; }
    public void addDPower(double add){ dPower = dPower + add; }






}//end of class