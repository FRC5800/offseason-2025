// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotFunil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MecherFunil extends Command {
  private PivotFunil pivotFunil;
  private double speed;

  /** Creates a new mecherFunil. */
  public MecherFunil(PivotFunil pivotFunil, double speed) {
    this.pivotFunil = pivotFunil;
    this.speed = speed;
    addRequirements(pivotFunil);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotFunil.run(speed);
    // SmartDashboard.putBoolean("Rodando", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivotFunil.run(0);
    // SmartDashboard.putBoolean("Rodando", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
