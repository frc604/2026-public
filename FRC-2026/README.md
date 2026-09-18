# 2026 RPC Code

2026 RPC onboard and offboard code for FRC604.

## Setup

### Install Python dependencies

- [Install](https://www.python.org/downloads/) Python 3.11 if it is not yet installed

- Run `pip3 install -r requirements.txt`

### To enable autoformatting when saving a file:

- [Install](https://marketplace.visualstudio.com/items?itemName=richardwillis.vscode-spotless-gradle) the `richardwillis.vscode-spotless-gradle` VS Code extension.

- In `File > Preferences > Settings`, search for `Format on Save` and enable it.

- If asked to select a formatter, choose `Spotless Gradle`.

## Code Structure

- `src/main/java/frc/quixlib`: General utilities to be used year-after-year
- `src/main/java/frc/robot`: Year-specific code
- `src/test/java/frc/...`: Unit tests for both `quixlib` and `robot`
- `offboard`: Code that doesn't run onboard the robot

## Commands

- `./gradlew spotlessApply` to format all code.
- `python3 offboard/quixpf/run_calibration.py` to run the camera extrinsic calibration
- `python3 offboard/quixpf/quixsam.py --local` to run the localizer in simulation
- View simulated cameras at:
    - http://localhost:1182 (left)
    - http://localhost:1184 (backleft)
    - http://localhost:1186 (backright)
    - http://localhost:1188 (right)

## AdvantageScope Setup

Do the following in AdvantageScope:
- To use the 3D robot model: `Help > Use Custom Assets Folder` and select `./advantagescope_assets`.
- To load the preconfigured layout: `File > Import Layout` and select `./advantagescope_layout.json`.
- In AdvantageScope `Settings`on Mac,`Help > Show Preferences` on Windows, make sure "Live Source" is set to "Network Tables 4 (AdvantageKit)"
