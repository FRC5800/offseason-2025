// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.XDrive;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Elevadancia;
import frc.robot.commands.Elevadancia;
import frc.robot.subsystems.Elevador;
import frc.robot.commands.Elevadancia;
import frc.robot.commands.ElevatorPID;
import frc.robot.subsystems.Elevador;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberComm;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MecherFunil;
import frc.robot.subsystems.PivotFunil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private XDrive xDrive = new XDrive();
  private Joystick controller = new Joystick(0);
  private Joystick controller02 = new Joystick(1);
  private Climber climber = new Climber();
  private Elevador elevador = new Elevador();
  private PivotFunil pivotFunil = new PivotFunil();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    xDrive.setDefaultCommand(new RunCommand(() -> xDrive.drive(-controller.getY(), controller.getX(), controller.getZ()), xDrive));
    elevador.setDefaultCommand(new Elevadancia(elevador, controller02));
    new JoystickButton(controller, 1).whileTrue(new MecherFunil(pivotFunil,1));
    new JoystickButton(controller, 2).whileTrue(new MecherFunil(pivotFunil, -1));
    new JoystickButton(controller, 3).whileTrue(new ClimberComm(climber, controller, 0.5));
    new JoystickButton(controller, 4).whileTrue(new ClimberComm(climber, controller, -0.5));
    new JoystickButton(controller02, 5).onTrue(new ElevatorPID(elevador,  0));
    new JoystickButton(controller02, 6).onTrue(new ElevatorPID(elevador, 1));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
