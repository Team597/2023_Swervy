// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Elevator;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorFX;


  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    this.elevatorFX = new TalonFX(Elevator.elevatorID);
    initMotor(elevatorFX);
    elevatorFX.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Pos ", elevatorFX.getSelectedSensorPosition());
  }

  public void set(double power){
    elevatorFX.set(ControlMode.PercentOutput, power);
  }
  public void stop(){
    elevatorFX.set(ControlMode.PercentOutput,0);
  }
  public void magicset(double target){
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
  }



}
