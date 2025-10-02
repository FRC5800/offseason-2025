// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XDrive;
import java.lang.annotation.Target;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoRotate extends Command {
    XDrive xDrive;
    boolean left;
    // List<Pose2dProto> coral_tags = new List<Pose2dStruct>();
    AprilTagFieldLayout apriltag_map = AprilTagFieldLayout.loadField(
        AprilTagFields.k2025ReefscapeAndyMark
    );
    ArrayList<Pose2d> reef_tag_poses = new ArrayList<Pose2d>();

    /** Creates a new AutoRotate. */
    public AutoRotate(XDrive xDrive) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.xDrive = xDrive;
        
        addRequirements(xDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        int[] reef_tags = { 6, 7, 8, 9, 10, 11 };

        int alliance_offset = DriverStation.getAlliance()
            .map(a -> a == Alliance.Red ? 0 : 11)
            .orElse(0);

        for (int i = 0; i < 6; i++) {
            var t = reef_tags[i] + alliance_offset;
            reef_tag_poses.add(apriltag_map.getTagPose(t).get().toPose2d());
        }

        double angle = this.xDrive
            .getPose2d()
            .nearest(reef_tag_poses)
            .getRotation().rotateBy(new Rotation2d(Math.PI))
            .getDegrees();

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
