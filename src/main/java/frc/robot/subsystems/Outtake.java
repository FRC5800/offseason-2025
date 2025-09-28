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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*Criando a classe do outtake */
public class Outtake extends SubsystemBase {
  /*Variáveis */

  //motor das rodas do outtake
  final SparkMax rodinhas; 

  /*Construtor do outtake com config do motor*/
    public Outtake () {
    
      rodinhas = new SparkMax(0, MotorType.kBrushless);  
       var rodinhasConfig = new SparkMaxConfig();
       rodinhasConfig.smartCurrentLimit(40);
       rodinhas.configure(rodinhasConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  
  @Override
  public void periodic() {
  }

}
