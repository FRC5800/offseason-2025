// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimberComm;
import frc.robot.subsystems.Climber;


public class RobotContainer {
  Joystick controle = new Joystick(0);
  Climber climber = new Climber();
 
  public RobotContainer() {
    configureBindings();
  }

  
  private void configureBindings() {
    climber.setDefaultCommand(new ClimberComm(climber, controle));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return null;
  }
}
