package com.frc6324.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/**
 * Executes a set of listed commands in sequential order, while also allowing "junction" commands to
 * run in between them.
 *
 * <p>This is essentially a wrapper over {@link SequentialCommandGroup}, however it offers a more
 * intuitive builder API.
 *
 * @see SequentialCommandGroup
 * @see Command
 * @see FollowPath
 */
public final class PathGroup extends Command {
  private final SequentialCommandGroup commands = new SequentialCommandGroup();
  private final FollowPath.Builder builder;

  /**
   * Creates a new path group, using the specified builder.
   *
   * @param builder The builder to build BLine paths with.
   */
  public PathGroup(FollowPath.Builder builder) {
    this.builder = builder;
  }

  /**
   * Creates a new path group using the specified builder, adding the given subsystems as
   * requirements.
   *
   * @param builder The builder to build BLine paths with.
   * @param requirements The requirements to add to this command.
   */
  public PathGroup(FollowPath.Builder builder, Subsystem... requirements) {
    this(builder);

    addRequirements(requirements);
  }

  /**
   * Adds a path to this path group.
   *
   * @param path The path to add. This will be loaded from the file {@code
   *     deploy/autos/paths/<path>.json}.
   * @return This group for easier method chaining.
   */
  public PathGroup nextPath(String path) {
    return nextPath(new Path(path));
  }

  /**
   * Adds an already-constructed path to this path group.
   *
   * @param path The path to add.
   * @return This group for easier method chaining.
   */
  public PathGroup nextPath(Path path) {
    Command followPath = builder.build(path);
    this.commands.addCommands(followPath);

    return this;
  }

  /**
   * Adds a command to be executed in this group after all previous paths and commands.
   *
   * @param command The command to add to the group.
   * @return This group for easier method chaining.
   */
  public PathGroup nextCommand(Command command) {
    return nextCommands(command);
  }

  /**
   * Adds a group of commands to be executed in this group after all previous paths and commands.
   *
   * @param commands The commands to add to the group.
   * @return This group for easier method chaining.
   */
  public PathGroup nextCommands(Command... commands) {
    this.commands.addCommands(commands);

    return this;
  }

  @Override
  public void initialize() {
    commands.initialize();
  }

  @Override
  public void execute() {
    commands.execute();
  }

  @Override
  public void end(boolean interrupted) {
    commands.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return commands.isFinished();
  }
}
