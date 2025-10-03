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

public class OuttakePreset extends SequentialCommandGroup {
  
  public OuttakePreset( Outtake outtake ) {
    
    addCommands( 
      new outtake_time(outtake, 0.3, 0.6),
      new outtake_time(outtake, 0.10, -0.6)
    );

  }
}
