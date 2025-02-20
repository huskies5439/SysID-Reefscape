// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Ascenseur;
import frc.robot.subsystems.Poignet;

public class RobotContainer {
  
  // private final Poignet poignet = new Poignet();
  private final Ascenseur ascenseur = new Ascenseur();

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final CommandXboxController manette = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

    SmartDashboard.putData(chooser);
    //Pour 2026, DONT stai mieux en teleop
    // chooser.addOption("Poignet - Quasistatique - Avancer", poignet.sysIdQuasistatic(Direction.kForward));
    // chooser.addOption("Poignet - Quasistatique - Reculer", poignet.sysIdQuasistatic(Direction.kReverse));
    // chooser.addOption("Poignet - Dynamique - Avancer", poignet.sysIdDynamic(Direction.kForward));
    // chooser.addOption("Poignet - Dynamique - Reculer", poignet.sysIdDynamic(Direction.kReverse));

    chooser.addOption("Ascenseur - Quasistatique - Avancer", ascenseur.sysIdQuasistatic(Direction.kForward));
    chooser.addOption("Ascenseur - Quasistatique - Reculer", ascenseur.sysIdQuasistatic(Direction.kReverse));
    chooser.addOption("Ascenseur - Dynamique - Avancer", ascenseur.sysIdDynamic(Direction.kForward));
    chooser.addOption("Ascenseur - Dynamique - Reculer", ascenseur.sysIdDynamic(Direction.kReverse));


  }

  private void configureBindings() {
    //Stai vraiment bien comme ca
    manette.a().whileTrue(ascenseur.sysIdQuasistatic(Direction.kForward));
    manette.b().whileTrue(ascenseur.sysIdQuasistatic(Direction.kReverse));
    manette.x().whileTrue(ascenseur.sysIdDynamic(Direction.kForward));
    manette.y().whileTrue(ascenseur.sysIdDynamic(Direction.kReverse));

  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
