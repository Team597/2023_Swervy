// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Positions;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndWrist extends ParallelCommandGroup {
  /** Creates a new ElevatorAndWrist. */
  public ElevatorAndWrist(ElevatorSubsystem ele, WristSubystem wrist, IntakeSubsystem intake, int pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double elePos = Positions.eHome;
    double wristPos = Positions.wHome;
    String intakeMode = intake.intakeMode();
    switch(pose){
      case 1: elePos = Positions.eHome;
              wristPos = Positions.wBothGround;
              break;
      case 2: if(intakeMode == "Cone"){
                elePos = Positions.eConeGroundPick;
                wristPos = Positions.wConeGroundPick;
                }else {
                  elePos = Positions.eHome;
                  wristPos = Positions.wBothGround;
                }
              break;
      case 3: if(intakeMode == "Cone"){
                elePos = Positions.eConeSubPick;
                wristPos = Positions.wConeSubPick;
                }else {
                  elePos = Positions.eCubeSubPick;
                  wristPos = Positions.wCubeSubPick;
                }
              break;
      case 4: if(intakeMode == "Cone"){
                elePos = Positions.eMidScore;
                wristPos = Positions.wConeMidScore;
                }else {
                  elePos = Positions.eMidScore;
                  wristPos = Positions.wCubeMidScore;
                }
              break;
      case 5: if(intakeMode == "Cone"){
                elePos = Positions.eHighScore;
                wristPos = Positions.wConeHighScore;
                }else {
                  elePos = Positions.eHighScore;
                  wristPos = Positions.wCubeHighScore;
                }
              break;
      default:  elePos = Positions.eHome;
                wristPos = Positions.wHome;
                break;
    }
    
    double timeDelay = 0;
    if(pose!= 0){
      timeDelay = 0.1;
    } else {
      timeDelay = 0.0;
    }

    addCommands(new MagicElevator(ele, elePos), new WaitMagicWrist(timeDelay, wrist, wristPos));
  }
}
