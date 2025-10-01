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
import frc.robot.subsystems.Outtake;
import frc.robot.commands.Elevadancia;
import frc.robot.commands.ElevatorPID;
import frc.robot.subsystems.Elevador;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ClimberComm;
// import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MexerFunil;
import frc.robot.commands.MoveOuttake;
import frc.robot.commands.Auto.AutoMove;
import frc.robot.commands.Auto.AutoRotate;
import frc.robot.commands.Auto.Autonomous_time;
import frc.robot.commands.Auto.Drivetimer;
import frc.robot.subsystems.PivotFunil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private XDrive xDrive = new XDrive();
  private Joystick controller = new Joystick(0);
  private Joystick controller02 = new Joystick(1);
  // private Climber climber = new Climber();
  private Elevador elevador = new Elevador();
  private PivotFunil pivotFunil = new PivotFunil();
  private Outtake outtake = new Outtake();
  private double height = 150;

  public RobotContainer() {
    SmartDashboard.getNumber("Altura do Elevador", height);
    configureBindings();
  }

  private void configureBindings() {
    xDrive.setDefaultCommand(new RunCommand(() -> xDrive.drive(-controller.getY(), controller.getX(), controller.getZ()), xDrive));
    new JoystickButton(controller, 4).onTrue(new InstantCommand(() -> xDrive.switchSpeed()));

    
    height = SmartDashboard.getNumber("Altura do Elevador", 150);

    // elevador.setDefaultCommand(new Elevadancia(elevador, controller02));
    elevador.setDefaultCommand(new ElevatorPID(elevador, controller02, 0));
    // new JoystickButt%on(controller02, 3).whileTrue(new MexerFunil(pivotFunil,1));
    // new JoystickButton(controller02, 4).whileTrue(new MexerFunil(pivotFunil, -1));
    // new JoystickButton(controller02, 1).whileTrue(new ClimberComm(climber, 0.5));
    // new JoystickButton(controller02, 2).whileTrue(new ClimberComm(climber, -0.5));
    new JoystickButton(controller02, 3).whileTrue(new MoveOuttake(outtake, 0.5));;
    new JoystickButton(controller02, 4).whileTrue(new MoveOuttake(outtake, -0.5));
    // new JoystickButton(controller02, 5).onTrue(new ElevatorPID(elevador,  2));
    // new JoystickButton(controller02, 6).onTrue(new ElevatorPID(elevador, 155));
    // new JoystickButton(controller, 1).whileTrue(new AutoRotate(xDrive, true));
    // new JoystickButton(controller, 3).onTrue(new MoveToTrajectory(xDrive, 1));    
    // new JoystickButton(controller, 4).onTrue(new MoveToTrajectory(xDrive, 6));    
  }

  
  public Command getAutonomousCommand() {
    //return new Drivetimer(xDrive, 2);
    // An example command will be run in autonomous
    return new Autonomous_time(xDrive, outtake);
  }
}