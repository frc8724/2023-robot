// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;

public class ClawColorSensor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  final double[] CUBE1_COLORS_RGB = { 0.20, 0.33, 0.47 };
  final double[] CUBE2_COLORS_RGB = { 0.23, 0.43, 0.33 };

  final double[] CONE_COLORS_RGB = { 0.37, 0.54, 0.08 };

  final double COLOR_SLOP = 0.03;

  Color detectedColor;

  /** Creates a new ClawColorSensor. */
  public ClawColorSensor() {
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    detectedColor = m_colorSensor.getColor();

    // SmartDashboard.putNumber("Color Red", detectedColor.red);
    // SmartDashboard.putNumber("Color Green", detectedColor.green);
    // SmartDashboard.putNumber("Color Blue", detectedColor.blue);

    SmartDashboard.putBoolean("Is Cone", isCone());
    SmartDashboard.putBoolean("Is Cube", isCube());
  }

  public boolean isCube() {
    return (Math.abs(detectedColor.red - CUBE1_COLORS_RGB[0]) < COLOR_SLOP &&
        Math.abs(detectedColor.green - CUBE1_COLORS_RGB[1]) < COLOR_SLOP &&
        Math.abs(detectedColor.blue - CUBE1_COLORS_RGB[2]) < COLOR_SLOP)
        ||
        (Math.abs(detectedColor.red - CUBE2_COLORS_RGB[0]) < COLOR_SLOP &&
            Math.abs(detectedColor.green - CUBE2_COLORS_RGB[1]) < COLOR_SLOP &&
            Math.abs(detectedColor.blue - CUBE2_COLORS_RGB[2]) < COLOR_SLOP);
  }

  public boolean isCone() {
    return Math.abs(detectedColor.red - CONE_COLORS_RGB[0]) < COLOR_SLOP &&
        Math.abs(detectedColor.green - CONE_COLORS_RGB[1]) < COLOR_SLOP &&
        Math.abs(detectedColor.blue - CONE_COLORS_RGB[2]) < COLOR_SLOP;
  }
}
