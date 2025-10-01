// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.XDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Autonomous_time extends SequentialCommandGroup {
  
  public Autonomous_time(XDrive xdrive, Outtake outtake) {
    
    addCommands( 
      new Drivetimer(null, 3.2),
      new outtake_time(outtake, 2)


    );

  }
}
