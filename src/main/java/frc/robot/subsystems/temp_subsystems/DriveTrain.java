// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.temp_subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  //Joysticks
  private Joystick controller = new Joystick(0);

  //Motor Controllers
  private SparkMax leftMotor = new SparkMax(0, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(1, MotorType.kBrushless);

  //Encoders
  private AbsoluteEncoder leftEncoder;
  private AbsoluteEncoder rightEncoder;

  //Drive Train
  private DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

  //data
  private Field2d field = new Field2d();

  //Simulation Objects
  private DifferentialDrivetrainSim diffDriveSim = new DifferentialDrivetrainSim(
    DCMotor.getNEO(2),
    10.71,
    5.708124999999999,
    60,
    Units.inchesToMeters(6),
    0.685,
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    //  Configuring the Spark Max Controllers
    var leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.smartCurrentLimit(60);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    var rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.smartCurrentLimit(60);
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Setting the objects of the encoders
    leftEncoder = leftMotor.getAbsoluteEncoder();
    rightEncoder = rightMotor.getAbsoluteEncoder();

    //  Publish the field on Dashboard
    SmartDashboard.putData(field);
  }

  public void drive() {
    diffDrive.arcadeDrive(-controller.getY(), -controller.getZ());
  }

  public double getLeft() {
    return leftEncoder.getPosition();
  }

  public double getRight() {
    return rightEncoder.getPosition();
  }
  
  public double getAvarage() {
    return getRight() + getLeft() / 2.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Avarage:", getAvarage());
  }

  @Override
  public void simulationPeriodic() {
    diffDriveSim.setInputs(leftMotor.get() * RobotController.getInputVoltage(), rightMotor.get() * RobotController.getInputVoltage());
    diffDriveSim.update(0.02);

    field.setRobotPose(diffDriveSim.getPose());
  }
}
