package frc.robot;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class RobotMath
{

  public static class Arm
  {

    /**
     * Convert {@link Angle} into motor {@link Angle}
     *
     * @param measurement Angle, to convert.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertAngleToSensorUnits(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) * ArmConstants.kArmReduction);
    }

    /**
     * Convert motor rotations {@link Angle} into usable {@link Angle}
     *
     * @param measurement Motor roations
     * @return Usable angle.
     */
    public static Angle convertSensorUnitsToAngle(Angle measurement)
    {
      return Rotations.of(measurement.in(Rotations) / ArmConstants.kArmReduction);

    }
  }

  public static class Elevator
  {

    /**
     * Convert {@link Distance} into {@link Angle}
     *
     * @param distance Distance, usually Meters.
     * @return {@link Angle} equivalent to rotations of the motor.
     */
    public static Angle convertDistanceToRotations(Distance distance)
    {
      return Rotations.of(distance.in(Meters) /
                          (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI) *
                          ElevatorConstants.kElevatorGearing);
    }

    /**
     * Convert {@link Angle} into {@link Distance}
     *
     * @param rotations Rotations of the motor
     * @return {@link Distance} of the elevator.
     */
    public static Distance convertRotationsToDistance(Angle rotations)
    {
      return Meters.of((rotations.in(Rotations) / ElevatorConstants.kElevatorGearing) *
                       (ElevatorConstants.kElevatorDrumRadius * 2 * Math.PI));
    }
  }
}
