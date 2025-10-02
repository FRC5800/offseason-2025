// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorPID;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.XDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Autonomous_time extends SequentialCommandGroup {
  
  public Autonomous_time(XDrive xdrive, Outtake outtake, Elevador elevador) {
    
    addCommands( 
      new Drivetimer(xdrive, 
      2.8),
      new WaitCommand(0.5),
      // new AutoMove(xdrive, true).withTimeout(6),
      new InstantCommand(() -> elevador.setTarget(160),elevador),
      new WaitCommand(2),
      new outtake_time(outtake, 1),
      new InstantCommand(() -> elevador.setTarget(0),elevador)
    );

  }
}
