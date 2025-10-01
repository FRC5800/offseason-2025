// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPID extends Command {
  /** Creates a new ElevatorPID. */
  Elevador elevador;
  PS4Controller controller;
  double target;

  public ElevatorPID(Elevador elevador, PS4Controller controller, double target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevador = elevador;
    this.controller = controller;
    this.target = target;
    addRequirements(elevador);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevador.setTarget(target);
    if(controller.getPOV() == 0)
      elevador.setTarget(160);
    else if(controller.getPOV() == 180)
      elevador.setTarget(5);
    else if(controller.getPOV() == 270)
      elevador.setTarget(75);
    else if(controller.getPOV() == 90)
      elevador.setTarget(125);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
