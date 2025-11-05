// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//oiiii n se assustem, tudo se importa sozinho com o quick fix!!

//criando a classe do climber
public class Climber extends SubsystemBase {
  private SparkMax climberMotor = new SparkMax(7, MotorType.kBrushless);

  public Climber() { //esse é o construtor dele
    var climberMotorMotorConfig = new SparkMaxConfig(); //pega o controlador dele, que é o SparkMax (o motor dele é o NEO)
    climberMotorMotorConfig.idleMode(IdleMode.kBrake); //IdleMode.kBrake segura a posição dele, ao invés de ir soltando aos poucos
    climberMotorMotorConfig.smartCurrentLimit(80); //define limite de corrente
    climberMotor.configure(climberMotorMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //metodo pra colocar/setar a velocidade dele
  public void run(double speed) {
    climberMotor.set(speed*0.8);
  }
}