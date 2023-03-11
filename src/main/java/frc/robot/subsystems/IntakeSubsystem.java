// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

/** Add your docs here. */
public class IntakeSubsystem extends SubsystemBase{
    private TalonSRX intakeTalon;
    private DoubleSolenoid intakeSolenoid;
    private String mode;

    
    public IntakeSubsystem(){
        this.intakeTalon = new TalonSRX(Intake.intakeID);
        this.intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Intake.solenoidIn, Intake.solenoidOut);
        intakeSolenoid.set(Value.kForward);
        mode = "Cone";
        intakeTalon.setNeutralMode(NeutralMode.Brake);
    }
    
    @Override
    public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putString("Intaking", mode);
    }

    public void driveIntake(double power){
        intakeTalon.set(ControlMode.PercentOutput, power);
    }

    public void toggleSolenoid(boolean dir){
        if(dir){
            intakeSolenoid.set(Value.kForward);
            mode = "Cone";
        }else{
            intakeSolenoid.set(Value.kReverse);
            mode = "Box";
        }
    }

    public String intakeMode(){
        return mode;
    }


}
