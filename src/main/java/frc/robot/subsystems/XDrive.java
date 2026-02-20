// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class XDrive extends SubsystemBase {
    // Controllers
    public SparkMax lf = new SparkMax(1, MotorType.kBrushless);
    public SparkMax rf = new SparkMax(2, MotorType.kBrushless);
    public SparkMax rb = new SparkMax(4, MotorType.kBrushless);
    public SparkMax lb = new SparkMax(3, MotorType.kBrushless);

    // Sensors
    public RelativeEncoder lfEncoder;
    public RelativeEncoder rfEncoder;
    public RelativeEncoder rbEncoder;
    public RelativeEncoder lbEncoder;
    public AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    VisionSystem visionSystem = new VisionSystem();
    public boolean isActive = true;

    // Modules speeds
    double FL = 0;
    double FR = 0;
    double BL = 0;
    double BR = 0;

    // 2D field
    public Field2d field = new Field2d();

    // Kinematics and pose estimators
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

    // PID and trajectory controllers
    PIDController yController = new PIDController(0.05, 0, 0);
    PIDController xController = new PIDController(1, 0, 0);
    ProfiledPIDController rController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(3, 1));
    public HolonomicDriveController controller = new HolonomicDriveController(xController, yController, rController);
    public PIDController rotationController = new PIDController(0.006, 0.001, 0.005);
    public PIDController movementController = new PIDController(0.05, 0, 0);

    // Variables to define the velocity of the robot
    double maxSpeed = 0.9;
    boolean turbo = true;
    double gyroSim = 0;

    // Constructor
    public XDrive() {
        // Reset gyro and poseEstimator
        poseEstimator.resetPose(new Pose2d());
        gyro.reset();

        //Configuring controllers
        var lfconfig = new SparkMaxConfig();
        lfconfig.idleMode(IdleMode.kBrake);
        lfconfig.smartCurrentLimit(60);
        lfconfig.disableFollowerMode();
        lf.configure(lfconfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        var rfconfig = new SparkMaxConfig();
        rfconfig.idleMode(IdleMode.kBrake);
        rfconfig.smartCurrentLimit(60);
        rfconfig.disableFollowerMode();
        rf.configure(rfconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var rbconfig = new SparkMaxConfig();
        rbconfig.idleMode(IdleMode.kBrake);
        rbconfig.smartCurrentLimit(60);
        rbconfig.disableFollowerMode();
        rb.configure(rbconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        var lbconfig = new SparkMaxConfig();
        lbconfig.idleMode(IdleMode.kBrake);
        lbconfig.smartCurrentLimit(60);
        lbconfig.disableFollowerMode();
        lb.configure(lbconfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Configuring encoders
        lfEncoder = lf.getEncoder();
        rfEncoder = rf.getEncoder();
        lbEncoder = lb.getEncoder();
        rbEncoder = rb.getEncoder();

        SmartDashboard.putData("field", field); // Put the field on the map

        // Configuring PIDControllers
        rotationController.setTolerance(0.4);
        rotationController.enableContinuousInput(0, 360);
        movementController.setTolerance(1);
    }

    public void periodic(){   
        SmartDashboard.putNumber("Max speed", maxSpeed);
        SmartDashboard.putNumber("Left front current", lf.getOutputCurrent());
        SmartDashboard.putNumber("Right front current", rf.getOutputCurrent());
        SmartDashboard.putNumber("Left back current", lb.getOutputCurrent());
        SmartDashboard.putNumber("Right back current", rb.getOutputCurrent());
        SmartDashboard.putNumber("gyro", gyro.getAngle());
        SmartDashboard.putNumber("PID Rotation", rotationController.getSetpoint());
        SmartDashboard.putData("pid rot", rotationController);
        visionSystem.getEstimatedPose().ifPresent(pose -> {
            poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
        });
        field.setRobotPose(
            poseEstimator.update(new Rotation2d(Units.degreesToRadians(-gyro.getAngle())),
            new MecanumDriveWheelPositions(ticksToMeter(lfEncoder.getPosition()), ticksToMeter(rfEncoder.getPosition()), ticksToMeter(lbEncoder.getPosition()), ticksToMeter(rbEncoder.getPosition())))
        );
    }

    // Method to make the drive more smooth
    double lerp(double a, double b, double f)  {
        return a + f * (b - a);
    }

    // Method to rotate in its own axis with pid
    public void autoRotate(){
        if (RobotBase.isSimulation()) {
            drive(0, 0, MathUtil.clamp(rotationController.calculate(gyroSim), -0.5, 0.5));
            return;
        }
        drive(0, 0, MathUtil.clamp(rotationController.calculate(gyro.getAngle()), -0.5, 0.5));
    }

    public void switchSpeed(){
        if(turbo){
            maxSpeed = 0.2;
            turbo = false;
        } else {
            maxSpeed = 0.9;
            turbo = true;
        }
    }

    public Pose2d getPose2d(){
        return poseEstimator.getEstimatedPosition();
    }

    public void resetController(){
        controller = new HolonomicDriveController(xController, yController, rController);
    }

    public void drive(double y, double x, double r) {       
        if(!isActive)
            return;

        if (RobotBase.isSimulation()) {
            
            poseEstimator.resetPose(new Pose2d(getPose2d().getTranslation().plus(new Translation2d(y*0.1, -x*0.1)), getPose2d().getRotation().rotateBy(new Rotation2d(r*0.05))));
            
            gyroSim += Math.toDegrees(r*0.05);
            return;
        }

        double botHeading = -Math.toRadians(gyro.getAngle() + 180);
        
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;
        double omega = r;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(r), 1);
        FL = lerp(FL, ((rotY + rotX + omega) / denominator) * maxSpeed, 0.1);
        FR = lerp(FR, ((rotY - rotX - omega) / denominator) * maxSpeed, 0.1);
        BL = lerp(BL, ((rotY - rotX + omega) / denominator) * maxSpeed, 0.1);
        BR = lerp(BR, ((rotY + rotX - omega) / denominator) * maxSpeed, 0.1);

        // Aplicando aos motores
        lf.set(FL);
        rf.set(FR);
        lb.set(BL);
        rb.set(BR);
    }

    public void driveRelative(double y, double x, double r) {
        if (RobotBase.isSimulation()) {
            poseEstimator.resetPose(getPose2d().transformBy(new Transform2d(y*0.1, -x*0.1, new Rotation2d(r*0.05))));
            gyroSim += Math.toDegrees(r*0.05);
            return;
        }

        double rotX = x;
        double rotY = y;
        double omega = r;

        double denominator = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(r), 1);
        FL = lerp(FL, ((rotY + rotX + omega) / denominator) * maxSpeed, 0.1);
        FR = lerp(FR, ((rotY - rotX - omega) / denominator) * maxSpeed, 0.1);
        BL = lerp(BL, ((rotY - rotX + omega) / denominator) * maxSpeed, 0.1);
        BR = lerp(BR, ((rotY + rotX - omega) / denominator) * maxSpeed, 0.1);

        // Aplicando aos motores
        lf.set(FL);
        rf.set(FR);
        lb.set(BL);
        rb.set(BR);
    }

    public double ticksToMeter(double ticks){
        // return ticks;
        return ticks * (Units.inchesToMeters(6)*Math.PI) / 14.75;
    }
}