// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.PivotFunil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class mecherFunil extends Command {

  PS4Controller controle;
  PivotFunil pivotFunil;

  /** Creates a new mecherFunil. */
  public mecherFunil(PivotFunil pivotFunil, PS4Controller controle) {
    this.controle = controle;
    this.pivotFunil = pivotFunil;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(controle.getLeftY()) > Constants.ZONA_MORTA_CONTROLE) {
      pivotFunil.abrirFunil(controle.getLeftY());
    } else {
      pivotFunil.abrirFunil(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
