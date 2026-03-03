# Design

## Autoware Agents

There are two Autoware agents — `aw_e2e.py` and `aw_privileged.py` — which send slightly modified information from the simulator to Autoware.  

The **E2E mode** uses raw sensor data from the simulator, while the **Privileged mode** uses ground-truth localization and object information instead. Since the simulator does not provide ground-truth prediction data, a **bicycle model prediction** is used, which is map-independent.

## Class Diagram

```mermaid
classDiagram
    class AutowareCarlaCppBridge
    note for AutowareCarlaCppBridge "Handles computationally heavy data <br/>(camera, LiDAR) and time synchronization <br/>between CARLA and Autoware"

    class AutonomousAgent

    class TUMROSBaseAgent {
    }
    TUMROSBaseAgent --|> AutonomousAgent

    class AutowarePriviligedAgent {
        - AutowareConverter _awp_converter
    }
    AutowarePriviligedAgent --|> TUMROSBaseAgent

    class AutowareE2EAgent {
        - AutowareConverter _awp_converter
    }
    AutowareE2EAgent --|> TUMROSBaseAgent

    class AutowareConverter {
        - AutowareToCarlaControl _control_converter
        - CarlaObjectsToAutoware _carla_objects_converter
    }

    AutowarePriviligedAgent --> AutowareConverter : uses
    AutowareE2EAgent --> AutowareConverter : uses

    class AutowareToCarlaControl
    class CarlaObjectsToAutoware

    AutowareConverter --> AutowareToCarlaControl : creates
    AutowareConverter --> CarlaObjectsToAutoware : creates
```

## Flow Diagram

The following flowchart illustrates the data flow within the framework.
The **AutowareE2EAgent** orchestrates the entire scenario, acting as a bridge to the **CARLA Scenario Runner**, and serves as the primary data interface between **CARLA** and **Autoware**.

The **AutowareCarlaCPPBridge**, introduced in **CARLA 0.9.16**, leverages CARLA’s **native ROS 2 DDS** interface to handle camera and LiDAR data. Additionally, Autoware time is continuously tracked within the **AutowareCarlaCPPBridge**, allowing multiple scenarios to be executed consecutively in Autoware without requiring a restart—since CARLA time resets to zero at the beginning of each new scenario.


![Flow Diagram](./io-diagramm.svg)

### Changes to CARLA
- Rename /clock to /carla/clock
- Rename /tf to /carla/f
- Reposition Traffic Light BBoxes see this https://github.com/carla-simulator/carla/issues/9445
