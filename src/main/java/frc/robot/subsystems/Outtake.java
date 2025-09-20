// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Criando a classe do outtake
public class Outtake extends SubsystemBase {

  final SparkFlex rodinhas; //motor das rodas do outtake

  //váriavel do sensor ultrassônico que vem do input do robo rio
  DigitalInput ultraSonicInput = new DigitalInput(1);

  public Outtake () {
    
    rodinhas = new SparkFlex(0, MotorType.kBrushless);

    var rodinhasConfig = new SparkFlexConfig();
    rodinhasConfig.smartCurrentLimit(40);
    rodinhas.configure(rodinhasConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity (double speed) {
      rodinhas.set(0.6);
  }

  public boolean getUltraSonic(){
    return ultraSonicInput.get();
  }
  /**
   * Example command factory method.
   *
   * @return a command 
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
        SmartDashboard.putBoolean("digitalInput", getUltraSonic());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
