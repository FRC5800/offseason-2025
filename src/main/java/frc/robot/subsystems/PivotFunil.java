// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PivotFunil extends SubsystemBase {

  private static final int LEAD_ID = 3;

  private static final double CLOSED_POS = 0;

  private static final double OPEN_POS = 0;

  WPI_TalonSRX pivotMotor = new WPI_TalonSRX(LEAD_ID);

   public PIDController pidControllerPivot = new PIDController(0.015, 0.01, 0.005);

  /** Creates a new PivotFunil. */
  public PivotFunil() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void abrirFunil()
}
