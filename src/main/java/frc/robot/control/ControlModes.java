package frc.robot.control;

import java.util.Map;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Auto.AutoMove;
import frc.robot.commands.Auto.AutoRotate;
import frc.robot.subsystems.Elevador;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.PivotFunil;
import frc.robot.subsystems.XDrive;

public class ControlModes{
  private XboxController driverController = new XboxController(0);
  private Joystick copilotController = new Joystick(1);
  
  private XDrive xDrive;
  private Outtake outtake;
  private Elevador elevador;
  private PivotFunil pivotFunil;

  public ControlModes(XDrive xDrive, Outtake outtake, Elevador elevador, PivotFunil pivotFunil){
    this.xDrive = xDrive;    
    this.outtake = outtake;    
    this.elevador = elevador;    
    this.pivotFunil = pivotFunil;
  }

  public static enum Mode{
    COMP1,
    COMP2,
    TEST,
  };

  // Map<Integer, Command> commandsComp1Driver = Map.of(
  //   2, new SequentialCommandGroup(
  //     new AutoRotate(xDrive),
  //     new AutoMove(xDrive, false)
  //   ),
    
  //   3, new SequentialCommandGroup(
  //     new AutoRotate(xDrive),
  //     new AutoMove(xDrive, true)
  //   ),
      
  //   4, new AutoMove(xDrive, true));

  // // Map<Subsystem, Command> defaultSubsystemCommandsComp1 = Map.of(
  // //   Elevador, new ElevatorPID(elevador, controller02, 0),
  // //   Outtake
  // // );  
    
  //   new JoystickButton(controller02, 3).whileTrue(new MoveOuttake(outtake, 0.5));
    
  //   new JoystickButton(controller02, 2).onTrue(new OuttakePreset(outtake));
    
  //   new JoystickButton(controller02, 4).whileTrue(new MoveOuttake(outtake, -0.5));
  //   new JoystickButton(controller02, 1).onTrue(new RunCommand(() -> outtake.switch_speed(), outtake));
  //   elevador.setDefaultCommand();
    
  
  //CODIGO NAO FUNCIONANDO, CASOTTI AJUDE, EU NAO SOU BOM EM JAVA ):



  public static Command ConfigureBindings(Mode mode){
    

    return new WaitCommand(1);
  }
}
