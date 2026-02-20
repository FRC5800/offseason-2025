// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.XDrive;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Outtake;
import frc.robot.commands.ElevatorPID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.MoveOuttake;
import frc.robot.commands.Auto.AutoMove;
import frc.robot.commands.Auto.AutoRotate;
import frc.robot.commands.Auto.Autonomous_time;
import frc.robot.commands.Auto.OuttakePreset;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  private XDrive xDrive = new XDrive();
  private XboxController controller = new XboxController(0);
  private Joystick controller02 = new Joystick(1);
  private Elevador elevador = new Elevador();
  private Outtake outtake = new Outtake();
  private double height = 150;

  public RobotContainer() {
    SmartDashboard.getNumber("Altura do Elevador", height);
    configureBindings();
  }

  private void configureBindings() {
    xDrive.setDefaultCommand(new RunCommand(() -> {
      double Yaxis = Math.abs(controller.getLeftY()) > 0.15 ? -controller.getLeftY() : 0;
      double Xaxis = Math.abs(controller.getLeftX()) > 0.15 ? controller.getLeftX() : 0;
      double Zaxis = Math.abs(controller.getRightX()) > 0.15 ? controller.getRightX() : 0;

      xDrive.drive(Yaxis, Xaxis, Zaxis);

      if(controller.getPOV() == 270)
        xDrive.driveRelative(0, -0.4, 0);
      else if(controller.getPOV() == 90)
        xDrive.driveRelative(0, 0.4, 0);
      else if(controller.getPOV() == 0)
        xDrive.driveRelative(0.9, 0, 0);
      else if(controller.getPOV() == 180)
        xDrive.driveRelative(-0.9, 0, 0);
    }, xDrive));

    new JoystickButton(controller, 4).whileTrue(new AutoMove(xDrive, true));
    new JoystickButton(controller, 3).whileTrue(new SequentialCommandGroup(
      new AutoRotate(xDrive),
      new AutoMove(xDrive, true))
    );
    new JoystickButton(controller, 2).whileTrue(new SequentialCommandGroup(
      new AutoRotate(xDrive),
      new AutoMove(xDrive, false))
    );

    height = SmartDashboard.getNumber("Altura do Elevador", 150);

    elevador.setDefaultCommand(new ElevatorPID(elevador, controller02, 0));

    new JoystickButton(controller02, 3).whileTrue(new MoveOuttake(outtake, 0.5));

    new JoystickButton(controller02, 2).onTrue(new OuttakePreset(outtake));

    new JoystickButton(controller02, 4).whileTrue(new MoveOuttake(outtake, -0.5));
    new JoystickButton(controller02, 1).onTrue(new RunCommand(() -> outtake.switch_speed(), outtake));
  }


  public Command getAutonomousCommand() {
    return new Autonomous_time(xDrive, outtake, elevador);
  }
}
