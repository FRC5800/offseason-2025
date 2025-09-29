// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.lang.annotation.Target;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRotate extends Command {
  XDrive xDrive;
  int idTarget;
  double angle;
  boolean estage;

  /** Creates a new AutoRotate. */
  public AutoRotate(XDrive xDrive, int idTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xDrive = xDrive;
    this.idTarget = idTarget;
    addRequirements(xDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    estage = idTarget == 1 || idTarget == 2;

    var alliance = DriverStation.getAlliance().orElse(Alliance.Red);
    if (alliance == DriverStation.Alliance.Blue)
      idTarget += 11;
    
    angle = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(idTarget).get().toPose2d().getRotation().getDegrees();
    if (estage)
      angle += 180;
      
    xDrive.rotationController.setSetpoint(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xDrive.autoRotate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xDrive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xDrive.rotationController.atSetpoint();
  }
}
