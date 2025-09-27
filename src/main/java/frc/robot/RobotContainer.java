// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.XDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ClimberComm;
import frc.robot.subsystems.Climber;


public class RobotContainer {
  private XDrive xDrive = new XDrive();
  private Joystick controller = new Joystick(0);
  private Climber climber = new Climber();
 
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    xDrive.setDefaultCommand(new RunCommand(() -> xDrive.drive(-controller.getY(), controller.getX(), controller.getZ()), xDrive));
    climber.setDefaultCommand(new ClimberComm(climber, controller));
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
