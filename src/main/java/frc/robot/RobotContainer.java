// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class RobotContainer {
  
  private final Poignet poignet = new Poignet();
  private final Ascenseur ascenseur = new Ascenseur();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData(chooser);

    chooser.addOption("Poignet - Quasistatique - Avancer", poignet.sysIdQuasistatic(Direction.kForward));
    chooser.addOption("Poignet - Quasistatique - Reculer", poignet.sysIdQuasistatic(Direction.kReverse));
    chooser.addOption("Poignet - Dynamique - Avancer", poignet.sysIdDynamic(Direction.kForward));
    chooser.addOption("Poignet - Dynamique - Reculer", poignet.sysIdDynamic(Direction.kReverse));

    chooser.addOption("Ascenseur - Quasistatique - Avancer", ascenseur.sysIdQuasistatic(Direction.kForward));
    chooser.addOption("Ascenseur - Quasistatique - Reculer", ascenseur.sysIdQuasistatic(Direction.kReverse));
    chooser.addOption("Ascenseur - Dynamique - Avancer", ascenseur.sysIdDynamic(Direction.kForward));
    chooser.addOption("Ascenseur - Dynamique - Reculer", ascenseur.sysIdDynamic(Direction.kReverse));


  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
