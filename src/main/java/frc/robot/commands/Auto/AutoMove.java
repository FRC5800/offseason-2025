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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XDrive;
import java.util.ArrayList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoMove extends Command {
  XDrive xDrive;
  int idTarget;
  double distance;
  boolean left;
  Pose2d target;
  ArrayList<Pose2d> reef_tag_poses = new ArrayList<Pose2d>();
  AprilTagFieldLayout apriltag_map = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeAndyMark
  );

  /** Creates a new AutoMove. */
  public AutoMove(XDrive xDrive, boolean left) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xDrive = xDrive;
    this.idTarget = idTarget;
    this.left = left;
    addRequirements(xDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      int[] reef_tags = { 6, 7, 8, 9, 10, 11 };

        int alliance_offset = DriverStation.getAlliance()
        .orElse(Alliance.Red) == Alliance.Red ? 0 : 11;

      for (int i = 0; i < 6; i++) {
          var t = reef_tags[i] + alliance_offset;
          reef_tag_poses.add(apriltag_map.getTagPose(t).get().toPose2d());
      }


    this.target = this.xDrive
        .getPose2d()
        .nearest(reef_tag_poses);

    double cosT = target
        .getRotation()
        .getCos();
    double senT = target
        .getRotation()
        .getSin();

    target = target
        .transformBy(
            new Transform2d(
                new Translation2d(0.17*-senT + 0.35 * cosT, 0.17*cosT),
                new Rotation2d()));
    
    // xDrive.movementController.setSetpoint(target.getTranslation().getNorm());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xDrive.driveRelative(0, 0.2, 0);
    this.xDrive.field.getObject("traj").setPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xDrive.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("automove dist", target.minus(this.xDrive.getPose2d()).getRotation().minus(target.getRotation()).getDegrees());
    return target.minus(this.xDrive.getPose2d()).getRotation().minus(target.getRotation()).getDegrees() < 5;
  }
}
