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
  double speed = 0.5;
  boolean turbo = true;
  
  //motor das rodas do outtake
  final TalonSRX rodinhas; 

  /*Construtor do outtake com config do motor*/
    public Outtake () {
    
      rodinhas = new TalonSRX(9);
      
    }
  
  @Override
  public void periodic() {
  }

  public void switch_speed(){
    if(turbo)
      turbo = !turbo;
    else
      turbo = !turbo;

  }

  public void run(double speed){
    if(turbo)
      rodinhas.set(TalonSRXControlMode.PercentOutput, speed);
    else
      rodinhas.set(TalonSRXControlMode.PercentOutput, speed-0.2);
  }

}
