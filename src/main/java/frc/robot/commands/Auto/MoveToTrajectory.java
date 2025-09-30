// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.List;
import java.util.concurrent.TransferQueue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToTrajectory extends Command {
  private XDrive xDrive;
  private Timer timer = new Timer();
  private Trajectory trajectory;
  private int idTarget;

  double maxSpeed = 3;
  double maxAcceleration = 1;

  /** Creates a new MoveToTrajectory. */
  public MoveToTrajectory(XDrive xDrive, int idTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xDrive = xDrive;
    this.idTarget = idTarget;
    addRequirements(xDrive);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = TrajectoryGenerator.generateTrajectory(
      xDrive.getPose2d(),
      List.of(),
      new Pose2d(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(idTarget).get().getTranslation().toTranslation2d(), xDrive.getPose2d().getRotation()),
      // new Pose2d(new Translation2d(xDrive.getPose2d().getX()+2, xDrive.getPose2d().getY()+1), xDrive.getPose2d().getRotation()),
      // AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(idTarget).get().toPose2d(),
      // xDrive.getPose2d().transformBy(new Transform2d(0, -2, new Rotation2d(Math.PI))),
      new TrajectoryConfig(maxSpeed, maxAcceleration)
    );

    xDrive.field.getObject("traj").setPoses(trajectory.getStates().stream().map(s -> s.poseMeters).toList());

    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xDrive.active = false;

    Trajectory.State goal = trajectory.sample(timer.get());

    ChassisSpeeds chassisSpeeds = xDrive.controller.calculate(xDrive.getPose2d(), goal, Rotation2d.fromDegrees(70));

    MecanumDriveWheelSpeeds wheelSpeeds = xDrive.kinematics.toWheelSpeeds(chassisSpeeds);

    xDrive.lf.set(wheelSpeeds.frontLeftMetersPerSecond/4.25);
    xDrive.rf.set(wheelSpeeds.frontRightMetersPerSecond/4.25);
    xDrive.lb.set(wheelSpeeds.rearLeftMetersPerSecond/4.25);
    xDrive.rb.set(wheelSpeeds.rearRightMetersPerSecond/4.25);

    // xDrive.lf.set(xDrive.lfPidController.calculate(xDrive.lfEncoder.getVelocity() * 60 * Math.PI * Units.inchesToMeters(6)));
    // xDrive.rf.set(xDrive.rfPidController.calculate(xDrive.rfEncoder.getVelocity() * 60 * Math.PI * Units.inchesToMeters(6)));
    // xDrive.lb.set(xDrive.lbPidController.calculate(xDrive.lbEncoder.getVelocity() * 60 * Math.PI * Units.inchesToMeters(6)));
    // xDrive.rb.set(xDrive.rbPidController.calculate(xDrive.rbEncoder.getVelocity() * 60 * Math.PI * Units.inchesToMeters(6)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xDrive.active = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}
