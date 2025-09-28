// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class XDrive extends SubsystemBase {
    public SparkMax lf = new SparkMax(1, MotorType.kBrushless);
    public SparkMax rf = new SparkMax(2, MotorType.kBrushless);
    public SparkMax rb = new SparkMax(4, MotorType.kBrushless);
    public SparkMax lb = new SparkMax(3, MotorType.kBrushless);

    public RelativeEncoder lfEncoder;
    public RelativeEncoder rfEncoder;
    public RelativeEncoder rbEncoder;
    public RelativeEncoder lbEncoder;

    public boolean active = true;

    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    double FL = 0;
    double FR = 0;
    double BL = 0;
    double BR = 0;

    double X = 0;
    double Y = 0;
    double R = 0;

    public Field2d field = new Field2d();

    public MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        new Translation2d(-0.25, 0.25),
        new Translation2d(0.25, 0.25),
        new Translation2d(-0.25, -0.25),
        new Translation2d(0.25, -0.25));
    MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
        kinematics,
        new Rotation2d(),
        new MecanumDriveWheelPositions(),
        new Pose2d()
    );

    public void autoRotate(){
        drive(0, 0, rotationController.calculate(gyro.getAngle()));
    }

    PIDController yController = new PIDController(0.02, 0, 0);
    PIDController xController = new PIDController(0.02, 0, 0);
    ProfiledPIDController rController = new ProfiledPIDController(1000, 0, 0, new TrapezoidProfile.Constraints(3, 1));
    public HolonomicDriveController controller = new HolonomicDriveController(xController, yController, rController);

    public PIDController rotationController = new PIDController(0.005, 0, 0);

    public PIDController lfPidController = new PIDController(0.01, 0, 0);
    public PIDController rfPidController = new PIDController(0.01, 0, 0);
    public PIDController lbPidController = new PIDController(0.01, 0, 0);
    public PIDController rbPidController = new PIDController(0.01, 0, 0);

    VisionSystem visionSystem = new VisionSystem();

    double lerp(double a, double b, double f)  {
        return a + f * (b - a);
    }

    public XDrive() {
        gyro.zeroYaw();
        var lfconfig = new SparkMaxConfig();
        lfconfig.idleMode(IdleMode.kBrake);
        lfconfig.smartCurrentLimit(80);
        lfconfig.disableFollowerMode();
        lf.configure(lfconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        var rfconfig = new SparkMaxConfig();
        rfconfig.idleMode(IdleMode.kBrake);
        rfconfig.smartCurrentLimit(80);
        rfconfig.disableFollowerMode();
        rf.configure(rfconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var rbconfig = new SparkMaxConfig();
        rbconfig.idleMode(IdleMode.kBrake);
        rbconfig.smartCurrentLimit(80);
        rbconfig.disableFollowerMode();
        rb.configure(rbconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var lbconfig = new SparkMaxConfig();
        lbconfig.idleMode(IdleMode.kBrake);
        lbconfig.smartCurrentLimit(80);
        lbconfig.disableFollowerMode();
        lb.configure(lbconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        lfEncoder = lf.getEncoder();
        rfEncoder = rf.getEncoder();
        lbEncoder = lb.getEncoder();
        rbEncoder = rb.getEncoder();

        SmartDashboard.putData("field", field);
        rotationController.setTolerance(1);
    }

    public Pose2d getPose2d(){
        return poseEstimator.getEstimatedPosition();
    }

    public void resetController(){
        controller = new HolonomicDriveController(xController, yController, rController);
    }

    public void drive(double y, double x, double r) {
        if(!active)
            return;
        double botHeading = -Math.toRadians(gyro.getAngle());
        
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;
        double omega = r;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(r), 1);
        FL = lerp(FL, ((rotY + rotX + omega) / denominator) * 0.7, 0.1);
        FR = lerp(FR, ((rotY - rotX - omega) / denominator) * 0.7, 0.1);
        BL = lerp(BL, ((rotY - rotX + omega) / denominator) * 0.7, 0.1);
        BR = lerp(BR, ((rotY + rotX - omega) / denominator) * 0.7, 0.1);

        // Aplicando aos motores
        lf.set(FL);
        rf.set(FR);
        lb.set(BL);
        rb.set(BR);
        
        
        // X = lerp(X, x, 0.1);
        // Y = lerp(Y, y, 0.1);
        // R = lerp(R, r, 0.1);
        // mecDrive.setMaxOutput(0.7);
        // mecDrive.driveCartesian(Y, X, R, new Rotation2d(-botHeading));
    }

    public double ticksToMeter(double ticks){
        // return ticks;
        return ticks * (Units.inchesToMeters(6)*Math.PI) / 14.75;
    }

    public void periodic(){   
        SmartDashboard.putNumber("gyro", gyro.getAngle());
        visionSystem.getEstimatedPose().ifPresent(pose -> {
            poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
        });
        field.setRobotPose(
            poseEstimator.update(new Rotation2d(Units.degreesToRadians(-gyro.getAngle())),
            new MecanumDriveWheelPositions(ticksToMeter(lfEncoder.getPosition()), ticksToMeter(rfEncoder.getPosition()), ticksToMeter(lbEncoder.getPosition()), ticksToMeter(rbEncoder.getPosition())))
        );
    }
}