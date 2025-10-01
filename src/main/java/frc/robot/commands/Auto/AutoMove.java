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
        SmartDashboard.putNumber("Drive pose X", 0);
        SmartDashboard.putNumber("Drive pose y", 0);
        SmartDashboard.putNumber("Drive pose angle", 0);
        addRequirements(xDrive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Pose2d simulation = new Pose2d(SmartDashboard.getNumber("Drive pose X", 0), SmartDashboard.getNumber("Drive pose Y", 0), new Rotation2d(SmartDashboard.getNumber("Drive pose angle", 0)));
        
        int[] reef_tags = { 6, 7, 8, 9, 10, 11 };
        
        int alliance_offset = DriverStation.getAlliance()
        .orElse(Alliance.Red) == Alliance.Red ? 0 : 11;
        
        for (int i = 0; i < 6; i++) {
            var t = reef_tags[i] + alliance_offset;
            reef_tag_poses.add(apriltag_map.getTagPose(t).get().toPose2d());
        }
        
        this.xDrive.field.setRobotPose(simulation);

        this.target = this.xDrive
            .field.getRobotPose()
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
                    new Translation2d(0.17*-senT, 0.17*cosT),
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
        Rotation2d angleTag = target.getRotation(); 
        Rotation2d angleRobot = new Rotation2d(Math.PI).plus(angleTag); 
        
        // Pose2d Offset = xDrive.getPose2d().relativeTo(target);

        // Rotation2d angleTriangle = angleRobot.minus(Offset.getRotation()); 

        // double distance = -angleTriangle.getSin()*Offset.getTranslation().getNorm();

        // SmartDashboard.putNumber("Distance axis", distance);
        // SmartDashboard.putNumber("Distance", Offset.getTranslation().getNorm());
        // SmartDashboard.putNumber("Angle Tag", angleTag.getDegrees());
        // SmartDashboard.putNumber("Angle Robot", angleRobot.getDegrees());
        // SmartDashboard.putNumber("Triangle angle", angleTriangle.getDegrees());

        // var distx = target.getTranslation().minus(xDrive.getPose2d().getTranslation()).rotateBy(xDrive.getPose2d().getRotation().unaryMinus()).getX();
        // SmartDashboard.putNumber("distancex", distx);

        
        // return true;

        Pose2d robotPose = xDrive.getPose2d();
        robotPose = new Pose2d(robotPose.getTranslation(), angleRobot);
        Pose2d tagPose = target;
    
        // normaliza ângulos para comparação: se diferença angular for ~0 ou ~pi => são paralelas/opostas
        double angleDiff = Math.abs(tagPose.getRotation().getRadians() - robotPose.getRotation().getRadians());
        // reduzir ao intervalo [0, pi]
        angleDiff = Math.abs(Math.atan2(Math.sin(angleDiff), Math.cos(angleDiff)));
    
        // threshold angular para considerar "paralelas" (em radianos). ex: 5 graus ~ 0.087 rad
        double parallelAngleThreshold = Math.toRadians(5.0);
    
        // vetor do tag até o robô no frame da tag: já nos dá componente lateral (Y) — a distância perpendicular
        Pose2d robotRelToTag = robotPose.relativeTo(tagPose);
        double perpSigned = robotRelToTag.getTranslation().getY();
        double perpAbs = Math.abs(perpSigned);
    
        SmartDashboard.putNumber("PerpDistance_signed_m", perpSigned);
        SmartDashboard.putNumber("PerpDistance_abs_m", perpAbs);
        SmartDashboard.putNumber("AngleDiff_deg", Math.toDegrees(angleDiff));
    
        // se NÃO são paralelas (ou seja, intersectam), a distância entre retas infinitas é zero
        if (angleDiff > parallelAngleThreshold && Math.abs(angleDiff - Math.PI) > parallelAngleThreshold) {
            SmartDashboard.putString("LineRelation", "intersecting (distance = 0)");
            // você pode optar por terminar aqui se quiser que intersectem; aqui consideramos não finalizado
            // return true; // opcional: se você quer terminar quando linhas se cruzam
        } else {
            SmartDashboard.putString("LineRelation", "parallel/opposed");
        }
    
        // Threshold de distância em metros para considerar "chegou" (ajuste conforme necessário)
        double finishThreshold = 0.07; // 3 cm
        boolean closeEnough = perpAbs < finishThreshold;
    
        SmartDashboard.putBoolean("PerpCloseEnough", closeEnough);
    
        return closeEnough;

    }
}
