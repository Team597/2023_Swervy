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
import frc.robot.Constants.Positions;
import frc.robot.Constants.Wrist;

public class WristSubystem extends SubsystemBase {
  private TalonSRX wristTalon;
  /** Creates a new WristSubystem. */

  public WristSubystem() {
    this.wristTalon = new TalonSRX(Wrist.wristID);
    initMotor(wristTalon);
    wristTalon.setNeutralMode(NeutralMode.Brake);
    wristTalon.setInverted(true);
    wristTalon.setSensorPhase(true);
    wristTalon.configReverseSoftLimitEnable(true);
    wristTalon.configReverseSoftLimitThreshold(1024);
    wristTalon.configClosedloopRamp(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
         SmartDashboard.putNumber("Wrist Pos ", wristTalon.getSelectedSensorPosition());
  }
  public void driveWrist(double power){
    wristTalon.set(ControlMode.PercentOutput, power);
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
    int kTimeoutMs = 30;

    
    motor.configFactoryDefault();
    
    motor.setInverted(false);
    motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs);
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
    motor.config_kF(kSlotIdx, 0.0, kTimeoutMs);
    motor.config_kP(kSlotIdx, 2.5, kTimeoutMs);
    motor.config_kI(kSlotIdx, 0.00002, kTimeoutMs);//0.000005
    motor.config_kD(kSlotIdx, 250.0, kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    motor.configMotionCruiseVelocity(30000, kTimeoutMs);//30000
    motor.configMotionAcceleration(800, kTimeoutMs);//1000
  }
}
