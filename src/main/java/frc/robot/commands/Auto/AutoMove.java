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
    Pose2d initialPose;
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
        SmartDashboard.putNumber("Drive pose X", 0);
        SmartDashboard.putNumber("Drive pose y", 0);
        SmartDashboard.putNumber("Drive pose angle", 0);
        SmartDashboard.putNumber("distY", 0);
        addRequirements(xDrive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialPose = xDrive.getPose2d();
        // Pose2d simulation = new Pose2d(SmartDashboard.getNumber("Drive pose X", 0), SmartDashboard.getNumber("Drive pose Y", 0), new Rotation2d(SmartDashboard.getNumber("Drive pose angle", 0)));
        
        int[] reef_tags = { 6, 7, 8, 9, 10, 11 };
        
        int alliance_offset = DriverStation.getAlliance()
        .orElse(Alliance.Red) == Alliance.Red ? 0 : 11;
        
        for (int i = 0; i < 6; i++) {
            var t = reef_tags[i] + alliance_offset;
            reef_tag_poses.add(apriltag_map.getTagPose(t).get().toPose2d());
        }
        
        // this.xDrive.field.setRobotPose(simulation);
        double mult = left ? -1 : 1;

        this.target = this.xDrive
            .getPose2d()
            .nearest(reef_tag_poses)
            .transformBy(new Transform2d(0, 0.17*mult, new Rotation2d()));
        
        // double cosT = target
        //     .getRotation()
        //     .getCos();
        // double senT = target
        //     .getRotation()
        //     .getSin();
        
       

        // target = target
        //     .transformBy(
        //         new Transform2d(
        //             new Translation2d(0.17*-senT*mult, 0.17*cosT*mult),
        //             new Rotation2d()));
        
        // xDrive.movementController.setSetpoint(target.getTranslation().getNorm());
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    
    double perpSigned;
    double perpAbs;
    double distY = 0;
    
    @Override
    public void execute() {
        this.xDrive.field.getObject("traj").setPose(target);
        
        Rotation2d angleTag = target.getRotation(); 
        Rotation2d angleRobot = new Rotation2d(Math.PI).plus(angleTag); 
        
        Pose2d robotPose = xDrive.getPose2d();
        robotPose = new Pose2d(robotPose.getTranslation(), angleRobot);
        Pose2d tagPose = target;
        
        // normaliza ângulos para comparação: se diferença angular for ~0 ou ~pi => são paralelas/opostas
        double angleDiff = Math.abs(tagPose.getRotation().getRadians() - robotPose.getRotation().getRadians());
        
        // reduzir ao intervalo [0, pi]
        angleDiff = Math.abs(Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff)));
        
        // vetor do tag até o robô no frame da tag: já nos dá componente lateral (Y) — a distância perpendicular
        Pose2d robotRelToTag = robotPose.relativeTo(tagPose);
        this.perpSigned = robotRelToTag.getTranslation().getY();
        this.perpAbs = Math.abs(perpSigned);
        
        SmartDashboard.putNumber("PerpDistance_signed_m", perpSigned);
        SmartDashboard.putNumber("PerpDistance_abs_m", perpAbs);
        SmartDashboard.putNumber("AngleDiff_deg", Math.toDegrees(angleDiff));

        var x = -perpSigned > 0 ? 0.25 : -0.25;
        this.distY = xDrive.getPose2d().getTranslation().plus(new Translation2d(0, x)).getDistance(target.getTranslation())-0.4;
        SmartDashboard.putNumber("distY", distY);
        var y = 0.25;

        if (closeEnoughX()) x = 0;
        if (closeEnoughY()) y = 0; 

        xDrive.driveRelative(y, x, 0);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        xDrive.drive(0, 0, 0);
    }

    boolean closeEnoughX() {
        return perpAbs < 0.07;
    }

    boolean closeEnoughY() {
        return distY < 0.2;
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {    
        boolean closeEnough = closeEnoughX() && closeEnoughY();
    
        SmartDashboard.putBoolean("PerpCloseEnough", closeEnough);
    
        return closeEnough || target.getTranslation().getDistance(initialPose.getTranslation()) > 3;

    }
}
