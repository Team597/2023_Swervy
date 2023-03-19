// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Positions;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorFX;
  private PowerDistribution pdh;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    this.elevatorFX = new TalonFX(Elevator.elevatorID);
    initMotor(elevatorFX);
    elevatorFX.setNeutralMode(NeutralMode.Brake);
    elevatorFX.configReverseSoftLimitEnable(false);
    elevatorFX.configReverseSoftLimitThreshold(0);
    pdh = new PowerDistribution(1, ModuleType.kRev);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double intakeCurrent = pdh.getCurrent(14);
    //SmartDashboard.putNumber("Elevator Draw", intakeCurrent);
    SmartDashboard.putNumber("Elevator Pos ", elevatorFX.getSelectedSensorPosition());
    SmartDashboard.putNumber("Elevator Switch", elevatorFX.isRevLimitSwitchClosed());
    if(elevatorFX.isRevLimitSwitchClosed()==1){
      elevatorFX.setSelectedSensorPosition(0);
    }
  }

  public void set(double power){
    if(elevatorFX.getSelectedSensorPosition()<=1200 && power < 0){
      power = 0;
    }
    elevatorFX.set(ControlMode.PercentOutput, power);
  }
  public void stop(){
    elevatorFX.set(ControlMode.PercentOutput,0);
  }

  //I hate command code
  public void magicset(int pose, Boolean isBox){
    double target = 0.0;
    switch(pose){
        case 1:   Constants.Intake.slowIntake = 0;
                  target = Positions.eHome;//GROUND PICK UP
                  break;
        case 2:   Constants.Intake.slowIntake = 0;
                  if(!isBox){ //CONE UPRIGHT GROUD PICK UP
                    target = Positions.eConeGroundPick;
                  } else {
                    target = Positions.eHome;
                  }
                  break;
        case 3:   Constants.Intake.slowIntake = 0;
                  if(!isBox){ // DUAL SUBSTATION PICK UP
                    target = Positions.eConeSubPick;
                  } else {
                    target = Positions.eCubeSubPick;
                  }
                  break;
        case 4:  Constants.Intake.slowIntake = 1;
                 if(!isBox){//LOW SCORE
                    target = Positions.eHome;
                  } else {
                    target = Positions.eHome;
                  }
                  break;
        case 5:   Constants.Intake.slowIntake = 1;
                  if(!isBox){//MID SCORE
                    target = Positions.eMidScore;
                  } else {
                    target = Positions.eHome;
                  }
                  break;
        case 6:   Constants.Intake.slowIntake = 1;      
                  if(!isBox){//HIGH SCORE
                    target = Positions.eHighScore;
                  } else {
                    target = Positions.eBoxHighScore;
                  }
                  break;
        default:  target = Positions.eHome;
                  Constants.Intake.slowIntake = 1;
                  break;
            }
    
    SmartDashboard.putNumber("Elevator Target", target);
    elevatorFX.set(ControlMode.MotionMagic,target);
  }




  /** Resets the motor completely */
  private void initMotor(TalonFX motor){
    int kSlotIdx = 0;
    int kPIDLoopIdx = 0;
    int kTimeoutMs = 30;

    motor.configFactoryDefault();
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);
    motor.configNeutralDeadband(0.001, kTimeoutMs);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

    motor.configNominalOutputForward(0, kTimeoutMs);
    motor.configNominalOutputReverse(0, kTimeoutMs);
    motor.configPeakOutputForward(1, kTimeoutMs);
    motor.configPeakOutputReverse(-1, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    motor.config_kF(kSlotIdx, 0.2, kTimeoutMs);
    motor.config_kP(kSlotIdx, 0.2, kTimeoutMs);
    motor.config_kI(kSlotIdx, 0.0, kTimeoutMs);
    motor.config_kD(kSlotIdx, 0.0, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    motor.configMotionCruiseVelocity(40000, kTimeoutMs);//35000
    motor.configMotionAcceleration(25000, kTimeoutMs);//20000

    /* Zero the sensor once on robot boot up */
    motor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);


    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(
      true, 30, 40, 0.2));
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration
    (true, 35, 40, 0.2));
  }



}
