// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.trajectoryLoader;

import java.nio.file.Path;
import edu.wpi.first.math.trajectory.Trajectory;

/////////////////////////////////////////////////////////////////////////////
/** Interface used to observe a call to loadJSONFiles() */
public interface LoadObserver {
  /** Method called when a Trajectory has been loaded successfully
   * @param fileName Name of the file the trajectory was loaded from
   * @param trajectory Trajectory object that was loaded
   */
  public void onTrajectoryCreated(String fileName, Trajectory trajectory);

  /** Method called on failure to load the contents of a file into a
   *  Trajectory object
   * @param filePath Path of the file that could not be loaded
   */
  public void onLoadFailed(Path filePath);
}