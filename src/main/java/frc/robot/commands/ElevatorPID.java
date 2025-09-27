// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPID extends Command {
  /** Creates a new ElevatorPID. */
  Elevador elevador;
  int target;

  public ElevatorPID(Elevador elevador, int target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevador = elevador;
    addRequirements(elevador);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevador.elevatorPIDMove(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevador.levantagem(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevador.inPosition();
  }
}
