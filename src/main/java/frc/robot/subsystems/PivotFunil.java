// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotFunil extends SubsystemBase {
  // Pivot constants
  private static final int LEAD_ID = 3;
  private static final double CLOSED_POS = -90;
  private static final double OPEN_POS = -35;
  private static final double PIVOT_LENGTH = 0.51804;
  private static final double PIVTO_MASS = 2.08691147;

  // Controllers
  private SparkMax pivotMotor = new SparkMax(LEAD_ID, MotorType.kBrushless);

  // PID Controller (temporary constants)
  public PIDController pidControllerPivot = new PIDController(0.015, 0.01, 0.005);

  // Mechanism canva
  private Mechanism2d pivotMech = new Mechanism2d(0.5, 0.5);
  private MechanismRoot2d pivotRoot = pivotMech.getRoot("Pivot root", 0.25, 0.4);
  private MechanismLigament2d pivot = new MechanismLigament2d("Pivot", PIVOT_LENGTH, OPEN_POS);

  // Simulation objects
  private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
    DCMotor.getCIM(1),
    100,
    SingleJointedArmSim.estimateMOI(PIVOT_LENGTH, PIVTO_MASS),
    PIVOT_LENGTH,
    Units.degreesToRadians(CLOSED_POS),
    Units.degreesToRadians(OPEN_POS),
    true,
    Units.degreesToRadians(OPEN_POS)
  );
    
  /** Creates a new PivotFunil. */
  public PivotFunil() {
    // Configuring controller
    var pivotMotorConfig = new SparkMaxConfig();
    pivotMotorConfig.idleMode(IdleMode.kBrake);
    pivotMotorConfig.smartCurrentLimit(60);
    pivotMotor.configure(pivotMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Append the pivot on the root
    pivotRoot.append(pivot);

    // Put the canva of the mechanism on Dashboard
    SmartDashboard.putData("Pivot Mech", pivotMech);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pivot angle", getAngle());
    SmartDashboard.putNumber("Motor", pivotMotor.get());
  }

  @Override
  public void simulationPeriodic() {
    // Set the input of the pivot and update simulation
    pivotSim.setInput(RobotController.getInputVoltage() * pivotMotor.get());
    pivotSim.update(0.02);

    // Set the angle of the mechanism of the pivot
    pivot.setAngle(getAngle());
  }

  public void run(double speed) {
    pivotMotor.set(speed * 0.5);
  }

  public double getAngle() {
    if (RobotBase.isSimulation())
      return Units.radiansToDegrees(pivotSim.getAngleRads());
    return 0;
  }
}
