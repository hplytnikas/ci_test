from enum import Enum


class LidarMode(Enum):
    OUSTER = "ouster"
    HESAI = "hesai"


class PerceptionMode(Enum):
    LIDAR_ONLY = "lidar_only"
    CAMERA_ONLY = "camera_only"
    SENSOR_FUSION = "sensor_fusion"


class ControllerMode(Enum):
    PURE_PURSUIT = "pure_pursuit"
    MPC_SAFE = "mpc_safe"
    MPC_RISKY = "mpc_risky"
    MPC_RAIN = "mpc_rain"


class CarMode(Enum):
    CASTOR = "castor"
    DUFOUR = "dufour"
