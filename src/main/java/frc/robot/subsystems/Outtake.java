// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/*Criando a classe do outtake */
public class Outtake extends SubsystemBase {
  /*Variáveis */

  //motor das rodas do outtake
  final TalonSRX rodinhas; 

  /*Construtor do outtake com config do motor*/
    public Outtake () {
    
      rodinhas = new TalonSRX(9);
      
    }
  
  @Override
  public void periodic() {
  }

  public void run(double speed){

    rodinhas.set(TalonSRXControlMode.PercentOutput, speed);


  }

}
