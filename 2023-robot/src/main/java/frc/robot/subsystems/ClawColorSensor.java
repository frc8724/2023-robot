// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

public class ClawColorSensor extends SubsystemBase {

  final double[] CUBE1_COLORS_RGB = { 0.20, 0.33, 0.47 };
  final double[] CUBE2_COLORS_RGB = { 0.23, 0.43, 0.33 };

  final double[] CONE_COLORS_RGB = { 0.37, 0.54, 0.08 };

  final double COLOR_SLOP = 0.03;

  Color detectedColor = Color.kAqua;
  int colorCount;
  Thread thread;
  double r;
  double g = 1.23;
  double b;

  /** Creates a new ClawColorSensor. */
  public ClawColorSensor() {
    // thread = new Thread(() -> Run());
    // thread.start();
  }

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  private void Run() {
    m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes13bit,
        ColorSensorMeasurementRate.kColorRate100ms, GainFactor.kGain3x);
    while (true) {
    }
  }

  private void ReadColorSensor() {
    Color c = m_colorSensor.getColor();
    setColor(c);
    // SmartDashboard.putNumber("Color x", c.red);
  }

  private synchronized Color getColor() {
    return detectedColor;
  }

  private synchronized int getCount() {
    return colorCount;
  }

  private synchronized void setColor(Color c) {
    detectedColor = c;
    r = c.red;
    g = c.green;
    b = c.blue;
    colorCount++;
  }

  private synchronized double getR() {
    return r;
  }

  private synchronized double getG() {
    return g;
  }

  private synchronized double getB() {
    return b;
  }

  /**
   * This method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    // Color c = getColor();
    // ReadColorSensor();
    // detectedColor = m_colorSensor.getColor();

    // SmartDashboard.putNumber("Color Red X", detectedColor.red);

    // SmartDashboard.putNumber("Color Red", getR());
    // SmartDashboard.putNumber("Color Green", getG());
    // SmartDashboard.putNumber("Color Blue", getB());

    // SmartDashboard.putNumber("Color Count", getCount());

    // SmartDashboard.putBoolean("Is Cone", isCone());
    // SmartDashboard.putBoolean("Is Cube", isCube());
  }

  public boolean isCube() {
    Color c = getColor();
    return (Math.abs(c.red - CUBE1_COLORS_RGB[0]) < COLOR_SLOP &&
        Math.abs(c.green - CUBE1_COLORS_RGB[1]) < COLOR_SLOP &&
        Math.abs(c.blue - CUBE1_COLORS_RGB[2]) < COLOR_SLOP)
        ||
        (Math.abs(c.red - CUBE2_COLORS_RGB[0]) < COLOR_SLOP &&
            Math.abs(c.green - CUBE2_COLORS_RGB[1]) < COLOR_SLOP &&
            Math.abs(c.blue - CUBE2_COLORS_RGB[2]) < COLOR_SLOP);
  }

  public boolean isCone() {
    Color c = getColor();
    return Math.abs(c.red - CONE_COLORS_RGB[0]) < COLOR_SLOP &&
        Math.abs(c.green - CONE_COLORS_RGB[1]) < COLOR_SLOP &&
        Math.abs(c.blue - CONE_COLORS_RGB[2]) < COLOR_SLOP;
  }
}
