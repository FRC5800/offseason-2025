// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoMove extends Command {
  XDrive xDrive;
  int idTarget;
  double distance;

  /** Creates a new AutoMove. */
  public AutoMove(XDrive xDrive, int idTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xDrive = xDrive;
    this.idTarget = idTarget;
    addRequirements(xDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(idTarget).get().toPose2d().transformBy(new Transform2d(new Translation2d(0.2, 0), new Rotation2d())).getTranslation().minus(xDrive.getPose2d().getTranslation()).getNorm();
    xDrive.movementController.setSetpoint(distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xDrive.autoDrive();;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xDrive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xDrive.movementController.atSetpoint();
  }
}
