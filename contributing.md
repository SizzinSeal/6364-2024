# Contributing
Everyone intending to contribute to the project should read this. It contains instructions on how to install all the necessary software, rules to follow, and coding conventions

## Tools
WPILib, requires you to install their own version of VSCode. Below are the various installers:
[Windows](https://packages.wpilib.workers.dev/installer/v2024.1.1/Win64/WPILib_Windows-2024.1.1.iso)
[Mac (ARM)](https://packages.wpilib.workers.dev/installer/v2024.1.1/macOSArm/WPILib_macOS-Arm64-2024.1.1.dmg)
[Mac (Intel)](https://packages.wpilib.workers.dev/installer/v2024.1.1/macOS/WPILib_macOS-Intel-2024.1.1.dmg)
[Linux](https://packages.wpilib.workers.dev/installer/v2024.1.1/Linux/WPILib_Linux-2024.1.1.tar.gz)

as well as the FRC game tools (windows only):
[Game Tools](https://packages.wpilib.workers.dev/game-tools/ni-frc-2024-game-tools_24.0.0_offline.iso)

## Discussion
Most of our discussions happen in person, with key points being sent to #frc-programming in Slack. If you are not in the slack, ask someone who is the next time you come in.

## Code Format
 * add `,"editor.formatOnSave": true` at the end of your workspace settings JSON file, which can be opened with the `workbench.action.openWorkspaceSettingsFile` command, so it formats the code whenever you hit save.
 * Use the default Red Hat Java formatter included in the `Language Support for Java` VScode extension.
 * Use 2 space indentation. Select it by clicking `Spaces: x` in the bottom right corner of the editor.

## Source Control
We use Git w/ GitHub for source control. The repo can be found [here](https://github.com/SizzinSeal/6364-2024). If you don't have access, it will 404. If you need access, contact Liam Teale on Slack or [email](mailto:liamteale6@gmail.com).

If you are new to Git, it's recommend installing [GitHub Desktop](https://desktop.github.com/) for an easy intro.

Small commits for things like tuning can be committed directly to the master branch. Other, larger commits implementing new features, updating dependencies, removing depencies, or installing dependencies need to be done on a seperate branch and merged in through Pull Requests.

Commit names should be short, and to the point. Based on the name of the commit you need to be able to tell what changed in that commit. Additional context can be added in the additional description section.

Branch names should be short, to the point, and follow this prefix:
`dep/branch_name` for anything to do with dependencies
`feature/branch_name` for new features. Supercedes `dep/`
`bugfix/branch_name` for bugfixes
If your branch does not fit any of these categories, it has no prefix.

:warning: **DO NOT** merge your pull request without proper testing and review from at least 1 other programmer

If you find a bug, or think a feature is needed, open an [Issue](https://github.com/SizzinSeal/6364-2024/issues/new)

## Building and Uploading

__**Build**__:
 1. Click the WPILib logo at the top right of the editor
 2. Click `Build Robot Code`

__**Upload (wired)**__:

 1. Connect your laptop to the roboRIO using a USB type-B cable
 2. Click the WPILib logo at the top right of the editor
 3. Click `Deploy Robot Code`
 4. Verify program uploaded successfully by checking the output terminal that pops up

__**Upload (wireless)**__:
 1. Connect to the `6364_Rex` wifi network. May take up to a minute to appear after startup
 2. Use password `pass123word` if prompted
 3. Click the WPILib logo at the top right of the editor
 4. Click `Deploy Robot Code`
 5. Verify program uploaded successfully by checking the output terminal that pops up

## Operating

:warning: As the station operator, it is **YOUR** responsibility to ensure the safety of those around you.

 1. Connect to the `6364_Rex` wifi network. May take up to a minute to appear after startup
 2. Use password `pass123word` if prompted
 3. Open the `FRC Driver Station` app
 4. Connect the controller with a micro-USB cable
 5. Verify controller connection in Driver Station
 6. Verify the area around the robot is clear and you are behind cover
 7. Select the necessary control mode (Teleop, Autonomous, Practice, Test)
 8. Announce "enabling" to the room and click the `enable` button
 9. Click `disable` to disable the robot or press space bar for an emergency stop