// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Intake;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase{
    private TalonSRX intakeTalon;
    private DoubleSolenoid intakeSolenoid;
    private String mode;
    //private PowerDistribution pdh;
    private Spark ledBlinky;


    
    public IntakeSubsystem(){
        this.intakeTalon = new TalonSRX(Intake.intakeID);
        this.intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Intake.solenoidIn, Intake.solenoidOut);
        intakeSolenoid.set(Value.kReverse);
        mode = "Box";
        intakeTalon.setNeutralMode(NeutralMode.Brake);
        //pdh = new PowerDistribution(1, ModuleType.kRev);
        ledBlinky= new Spark(0);
        blinky(1);
        intakeTalon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat,5000);
        intakeTalon.setStatusFramePeriod(StatusFrame.Status_1_General,5000);
        //elevatorFX.setStatusFramePeriod(StatusFrame.Status_10_MotionMagic,5000);
        intakeTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1,5000);
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putString("Intaking", mode);
    }

    public void driveIntake(double power){
        //double intakeCurrent = pdh.getCurrent(5);
        //SmartDashboard.putNumber("Intake Draw", intakeCurrent);
        if(Constants.Intake.slowIntake==1 && power > 0){
            power *= 0.1;
            
        }
        intakeTalon.set(ControlMode.PercentOutput, power);
    }

    public void toggleSolenoid(boolean dir){
        if(dir){
            intakeSolenoid.set(Value.kForward);
            mode = "Cone";
            blinky(2);
        }else{
            intakeSolenoid.set(Value.kReverse);
            mode = "Box";
            blinky(1);
        }
    }

    public void blinky(int casa){
        switch(casa){
            case 1: ledBlinky.set(0.17); //Box 
            break;
            case 2: ledBlinky.set(-0.03); //Cone
            break;
            case 3: ledBlinky.set(0.15); //Dual Station Cone
            break;
        }
    }


    public boolean isBox(){
        if (mode=="Cone"){
            return false;
        } else if(mode == "Box"){
            return true;
        }
        return false;
    }
    public double intakeDouble(){
        double returny = 0.0;
        if (mode=="Cone"){
            returny = 0.0;
        } else if(mode == "Box"){
            returny = 1.0;
        }
        return returny;
    }
    public String intakeMode(){
        return mode;
    }


}
