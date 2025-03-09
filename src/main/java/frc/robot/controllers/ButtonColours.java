// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controllers;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.Launchpad;

/** Add your docs here. */
public class ButtonColours {

    Launchpad m_launchpad;

    public ButtonColours (Launchpad launchpad){
        m_launchpad = launchpad;
    }
    public static final Color HPPressed = Color.kYellow;
    public static final Color CoralLevelColour = Color.kPurple;
    public static final Color AlgaeColour = Color.kGreen;
    public static final Color ScoreCoral = Color.kCyan;
    public static final Color HP = Color.kRed;
    public static final Color IsPressed = Color.kOrange;
    public static final Color notSelectedColour = Color.kWhite;
    public static final Color selectedColour = Color.kBlue;


    }
    

