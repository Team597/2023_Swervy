// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.WristSubystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorAndWrist extends ParallelCommandGroup {
  /** Creates a new ElevatorAndWrist. */
  public ElevatorAndWrist(ElevatorSubsystem ele, WristSubystem wrist, BooleanSupplier isBox, int pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    /*double elePos = Positions.eHome;
    double wristPos = Positions.wHome;
    String intakeMode = "Cone";

    
    switch(pose){
      case 1: elePos = Positions.eHome;//GROUND PICK UP
              wristPos = Positions.wBothGround;
              break;
      case 2: if(intakeMode == "Cone"){ //CONE UPRIGHT GROUD PICK UP
                elePos = Positions.eConeGroundPick;
                wristPos = Positions.wConeGroundPick;
                }else {
                  elePos = Positions.eHome;
                  wristPos = Positions.wBothGround;
                }
              break;
      case 3: if(intakeMode == "Cone"){ // DUAL SUBSTATION PICK UP
                elePos = Positions.eConeSubPick;
                wristPos = Positions.wConeSubPick;
                }else {
                  elePos = Positions.eCubeSubPick;
                  wristPos = Positions.wCubeSubPick;
                }
              break;
      case 4: if(intakeMode == "Cone"){//MID SCORE
                elePos = Positions.eMidScore;
                wristPos = Positions.wConeMidScore;
                }else {
                  elePos = Positions.eHome;
                  wristPos = Positions.wCubeMidScore;
                }
              break;
      case 5: if(intakeMode == "Cone"){//HIGH SCORE
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
    */
    double timeDelay = 0;
    if(pose!= 0){
      timeDelay = 0.1;
    } else {
      timeDelay = 0.0;
    }
    addCommands(new MagicElevator(ele, pose, isBox), new WaitMagicWrist(timeDelay, wrist, pose, isBox));
  }
}
