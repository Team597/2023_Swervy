// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Positions;
import frc.robot.Constants.Wrist;

public class WristSubystem extends SubsystemBase {
  private TalonSRX wristTalon;
  private double wristInc;
  /** Creates a new WristSubystem. */

  public WristSubystem() {
    this.wristTalon = new TalonSRX(Wrist.wristID);
    initMotor(wristTalon);
    wristTalon.setNeutralMode(NeutralMode.Brake);
    wristTalon.setInverted(true);
    wristTalon.setSensorPhase(true);
    wristTalon.configReverseSoftLimitEnable(false);
    //wristTalon.configReverseSoftLimitThreshold(1024);
    wristTalon.configClosedloopRamp(0.1);
    wristInc = Constants.Positions.wHome;
    wristTalon.overrideLimitSwitchesEnable(false);


    wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,5000);
    wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer,5000);
    wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc,5000);
    wristTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1,5000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
         SmartDashboard.putNumber("Wrist Pos ", wristTalon.getSelectedSensorPosition());
         if(limitSwitched()){
            wristTalon.setSelectedSensorPosition(Constants.Positions.wHome-220);
            //wristInc = 2340;
        }
  }

  public boolean limitSwitched(){
    if(wristTalon.isRevLimitSwitchClosed()==1){
      return true;
    } else{
      return false;
    }
  }

  public void driveWrist(double power){
    wristTalon.set(ControlMode.PercentOutput, power);
  }

  public void incMagic(double s_add){
    wristInc += s_add;
    wristTalon.set(ControlMode.MotionMagic, wristInc);
  }

//I hate command based coding
  public void magicset(int pose, boolean isBox){
    double target = Positions.wHome;
    switch(pose){
      case 1:   target = Positions.wBothGround;//GROUND PICK UP
                break;
      case 2:   if(!isBox){ //CONE UPRIGHT GROUD PICK UP
                  target = Positions.wConeGroundPick;
                } else {
                  target = Positions.wBothGround;
                }
                break;
      case 3:   if(!isBox){ // DUAL SUBSTATION PICK UP
                  target = Positions.wConeSubPick;
                } else {
                  target = Positions.wCubeSubPick;
                }
                break;
      case 4: if(!isBox){//LOW SCORE
                  target = Positions.wConeLowScore;
                } else {
                  target = Positions.wBoxLowScore;
                }
                break;
      case 5:   if(!isBox){//MID SCORE
                  target = Positions.wConeMidScore;
                } else {
                  target = Positions.wCubeMidScore;
                }
                break;
      case 6:   if(!isBox){//HIGH SCORE
                  target = Positions.wConeHighScore;
                } else {
                  target = Positions.wCubeHighScore;
                }
                break;
      default:  target = Positions.wHome;
                break;
    }

    SmartDashboard.putNumber("Wrist Target", target);
    wristTalon.set(ControlMode.MotionMagic, target);
  }


  
  /** Resets the motor completely */
  private void initMotor(TalonSRX motor){
    int kSlotIdx = 0;
    int kPIDLoopIdx = 0;
    int kTimeoutMs = 40;

    
    motor.configFactoryDefault();
    
    motor.setInverted(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs);
    motor.configNeutralDeadband(0.001, kTimeoutMs);

    /* Set relevant frame periods to be at least as fast as periodic rate */
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
    motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20, kTimeoutMs);

    motor.configNominalOutputForward(0, kTimeoutMs);
    motor.configNominalOutputReverse(0, kTimeoutMs);
    motor.configPeakOutputForward(1, kTimeoutMs);
    motor.configPeakOutputReverse(-1, kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    motor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
    motor.config_kF(kSlotIdx, 0.0, kTimeoutMs);
    motor.config_kP(kSlotIdx, 1.0, kTimeoutMs);
    motor.config_kI(kSlotIdx, 0.000025, kTimeoutMs);//0.00002
    motor.config_kD(kSlotIdx, 250.0, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    motor.configMotionCruiseVelocity(30000, kTimeoutMs);//30000
    motor.configMotionAcceleration(2000, kTimeoutMs);//1000
  }
}
