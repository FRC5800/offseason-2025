// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Outtake;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveOuttake extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //criando variáavel da classe outtake e velocidade
  private Outtake outtake;
  private double speed;

  //construtor do comando 
  public MoveOuttake(Outtake outtake, double speed) {
    this.outtake = outtake;
     this.speed = speed;
    addRequirements(outtake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    outtake.run(speed);

  //atribuindo o método que define a velocidade
  }

  @Override
  public void end(boolean interrupted) {
    outtake.run(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
