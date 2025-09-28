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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
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
  private static final int LEAD_ID = 8;
  private static final double CLOSED_POS = -90;
  private static final double OPEN_POS = -35;
  private static final double PIVOT_LENGTH = 0.51804;
  private static final double PIVTO_MASS = 2.08691147;

  // Controllers
  private WPI_TalonSRX pivotMotor = new WPI_TalonSRX(LEAD_ID);

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
    
  // Pivot position
  private Pose3d pivotPose;
  private Pose3d basePose;

  // Array publisher
  private StructArrayPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Pivot position", Pose3d.struct).publish();
  private StructArrayPublisher<Pose3d> basePosePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("Base pivot position", Pose3d.struct).publish();

  // temp publhiser
  private StructArrayPublisher<Pose3d> tempPublisher = NetworkTableInstance.getDefault().getStructArrayTopic("temp position", Pose3d.struct).publish();

  /** Creates a new PivotFunil. */
  public PivotFunil() {
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

    // temp pose
    Pose3d tempPose = new Pose3d(new Translation3d(0, 0, 0.93980), new Rotation3d());

    // Pivot pose
    pivotPose = new Pose3d(new Translation3d(-0.33, 0, 0.81875), new Rotation3d(0, -Units.degreesToRadians(getAngle() + 35), 0));
    basePose = new Pose3d(new Translation3d(), new Rotation3d());

    // Update publishers
    posePublisher.set(new Pose3d[]{ pivotPose });
    basePosePublisher.set(new Pose3d[] { basePose });
    tempPublisher.set(new Pose3d[]{ tempPose });
  }

  public void run(double speed) {
    pivotMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getAngle() {
    if (RobotBase.isSimulation())
      return Units.radiansToDegrees(pivotSim.getAngleRads());
    return 0;
  }
}
