"""Interactive keyboard teleoperation node for Robot 1.

Supports Arrow keys, French AZERTY (Z/Q/S/D), and English QWERTY (W/A/S/D)
layouts with automatic deadman stop timeout tuned for the 1.40 m x 1.00 m
Robogames competition arena.
"""

from dataclasses import dataclass
import select
import sys
import termios
import tty
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# ROS 2 Communication Constants
NODE_NAME: str = "robot1_keyboard_teleop"
CMD_VEL_TOPIC: str = "/cmd_vel"
PUBLISHER_QUEUE_SIZE: int = 10

# Velocity Limits & Default Speeds Tuned for 1.40 m x 1.00 m Arena
DEFAULT_LINEAR_SPEED_MPS: float = 0.20
DEFAULT_ANGULAR_SPEED_RADPS: float = 0.90
MIN_LINEAR_SPEED_MPS: float = 0.05
MAX_LINEAR_SPEED_MPS: float = 0.75
MIN_ANGULAR_SPEED_RADPS: float = 0.25
MAX_ANGULAR_SPEED_RADPS: float = 2.80
SPEED_SCALE_INCREMENT: float = 1.15
SPEED_SCALE_DECREMENT: float = 0.85

# Timing & Watchdog Constants (Seconds)
KEYBOARD_POLL_TIMEOUT_SEC: float = 0.05
ESCAPE_SEQUENCE_TIMEOUT_SEC: float = 0.02
# Must exceed typical OS keyboard initial auto-repeat delay (~0.50 s)
DEADMAN_WATCHDOG_TIMEOUT_SEC: float = 0.65

# Raw Terminal Key Constants
CTRL_C_KEY: str = "\x03"
ESC_KEY: str = "\x1b"
ESCAPE_BRACKET_PREFIX: str = "["
ARROW_UP_TOKEN: str = "ARROW_UP"
ARROW_DOWN_TOKEN: str = "ARROW_DOWN"
ARROW_RIGHT_TOKEN: str = "ARROW_RIGHT"
ARROW_LEFT_TOKEN: str = "ARROW_LEFT"

ARROW_ESCAPE_MAP: Dict[str, str] = {
    "A": ARROW_UP_TOKEN,
    "B": ARROW_DOWN_TOKEN,
    "C": ARROW_RIGHT_TOKEN,
    "D": ARROW_LEFT_TOKEN,
}


@dataclass(frozen=True)
class MotionDirection:
    """Immutable normalized velocity command multipliers.

    Attributes:
        linear_sign: Multiplier in [-1.0, 1.0] applied to linear X speed.
        angular_sign: Multiplier in [-1.0, 1.0] applied to angular Z speed.
    """

    linear_sign: float
    angular_sign: float


STOP_DIRECTION: MotionDirection = MotionDirection(linear_sign=0.0, angular_sign=0.0)

KEY_DIRECTION_BINDINGS: Dict[str, MotionDirection] = {
    ARROW_UP_TOKEN: MotionDirection(linear_sign=1.0, angular_sign=0.0),
    ARROW_DOWN_TOKEN: MotionDirection(linear_sign=-1.0, angular_sign=0.0),
    ARROW_LEFT_TOKEN: MotionDirection(linear_sign=0.0, angular_sign=1.0),
    ARROW_RIGHT_TOKEN: MotionDirection(linear_sign=0.0, angular_sign=-1.0),
    "z": MotionDirection(linear_sign=1.0, angular_sign=0.0),
    "w": MotionDirection(linear_sign=1.0, angular_sign=0.0),
    "s": MotionDirection(linear_sign=-1.0, angular_sign=0.0),
    "q": MotionDirection(linear_sign=0.0, angular_sign=1.0),
    "a": MotionDirection(linear_sign=0.0, angular_sign=1.0),
    "d": MotionDirection(linear_sign=0.0, angular_sign=-1.0),
    "e": MotionDirection(linear_sign=1.0, angular_sign=-0.6),
    "r": MotionDirection(linear_sign=1.0, angular_sign=0.6),
}

STOP_KEYS: Tuple[str, ...] = (" ", "k")
SPEED_UP_KEYS: Tuple[str, ...] = ("+", "=")
SPEED_DOWN_KEYS: Tuple[str, ...] = ("-", "_")
QUIT_KEYS: Tuple[str, ...] = (CTRL_C_KEY, "x")

HELP_BANNER: str = """
====================================================================
  Robot 1 - Keyboard Teleoperation (Hokuyo URG-04LX Duplo Test)
====================================================================
  Drive Controls (Arrow Keys, AZERTY Z/Q/S/D, or QWERTY W/A/S/D):
    [Up]    or [Z / W] : Move Forward  (+X)
    [Down]  or [S]     : Move Backward (-X)
    [Left]  or [Q / A] : Rotate Left   (+Yaw)
    [Right] or [D]     : Rotate Right  (-Yaw)
    [R] / [E]          : Arc Forward-Left / Forward-Right
    [Space] or [K]     : Immediate Stop

  Speed Adjustment:
    [+] or [=]         : Increase linear & angular speed (+15%)
    [-] or [_]         : Decrease linear & angular speed (-15%)

  Exit:
    [X] or [Ctrl+C]    : Stop robot and exit cleanly
====================================================================
"""


def clamp_value(value: float, lower_bound: float, upper_bound: float) -> float:
    """Clamp a floating-point value within inclusive [lower_bound, upper_bound].

    Args:
        value: Input scalar value to constrain.
        lower_bound: Minimum allowed value.
        upper_bound: Maximum allowed value.

    Returns:
        Clamped floating-point scalar.
    """
    return max(lower_bound, min(upper_bound, value))


def read_key_token(file_descriptor: int, terminal_settings: List[object]) -> str:
    """Read a single raw keypress or ANSI arrow escape token with timeout.

    Args:
        file_descriptor: Standard input UNIX file descriptor.
        terminal_settings: Saved `termios` attribute list restored after read.

    Returns:
        Normalized lowercase key token, arrow token constant, or empty string
        when no key was pressed within `KEYBOARD_POLL_TIMEOUT_SEC`.
    """
    tty.setraw(file_descriptor)
    try:
        readable, _, _ = select.select(
            [sys.stdin], [], [], KEYBOARD_POLL_TIMEOUT_SEC
        )
        if not readable:
            return ""

        first_char: str = sys.stdin.read(1)
        if first_char != ESC_KEY:
            return first_char.lower()

        follow_ready, _, _ = select.select(
            [sys.stdin], [], [], ESCAPE_SEQUENCE_TIMEOUT_SEC
        )
        if not follow_ready:
            return ESC_KEY

        second_char: str = sys.stdin.read(1)
        if second_char == ESCAPE_BRACKET_PREFIX:
            third_ready, _, _ = select.select(
                [sys.stdin], [], [], ESCAPE_SEQUENCE_TIMEOUT_SEC
            )
            if third_ready:
                third_char: str = sys.stdin.read(1)
                return ARROW_ESCAPE_MAP.get(third_char, "")
        return ""
    finally:
        termios.tcsetattr(file_descriptor, termios.TCSADRAIN, terminal_settings)


class KeyboardTeleopNode(Node):
    """ROS 2 publisher node translating keyboard commands into `/cmd_vel` Twist."""

    def __init__(self) -> None:
        """Initialize the `/cmd_vel` publisher and default speed state."""
        super().__init__(NODE_NAME)
        self._cmd_vel_publisher = self.create_publisher(
            Twist, CMD_VEL_TOPIC, PUBLISHER_QUEUE_SIZE
        )
        self._linear_speed_mps: float = DEFAULT_LINEAR_SPEED_MPS
        self._angular_speed_radps: float = DEFAULT_ANGULAR_SPEED_RADPS
        self._active_direction: MotionDirection = STOP_DIRECTION
        self._last_command_time_sec: float = self._now_seconds()

    def _now_seconds(self) -> float:
        """Return monotonic wall-clock time in seconds from the node clock.

        Returns:
            Current timestamp in seconds.
        """
        nanoseconds: int = self.get_clock().now().nanoseconds
        return float(nanoseconds) * 1e-9

    def publish_direction(self, direction: MotionDirection) -> None:
        """Publish a `Twist` message scaled by the current speed settings.

        Args:
            direction: Immutable normalized linear and angular multipliers.
        """
        twist_msg = Twist()
        twist_msg.linear.x = direction.linear_sign * self._linear_speed_mps
        twist_msg.angular.z = direction.angular_sign * self._angular_speed_radps
        self._cmd_vel_publisher.publish(twist_msg)

    def adjust_speeds(self, scale_factor: float) -> None:
        """Scale linear and angular speeds within configured arena safety bounds.

        Args:
            scale_factor: Multiplicative scaling ratio applied to both speeds.
        """
        self._linear_speed_mps = clamp_value(
            self._linear_speed_mps * scale_factor,
            MIN_LINEAR_SPEED_MPS,
            MAX_LINEAR_SPEED_MPS,
        )
        self._angular_speed_radps = clamp_value(
            self._angular_speed_radps * scale_factor,
            MIN_ANGULAR_SPEED_RADPS,
            MAX_ANGULAR_SPEED_RADPS,
        )
        self.print_status_line()

    def print_status_line(self) -> None:
        """Print current linear and angular speed targets to stdout."""
        sys.stdout.write(
            f"\rSpeed -> linear: {self._linear_speed_mps:.2f} m/s | "
            f"angular: {self._angular_speed_radps:.2f} rad/s    \n"
        )
        sys.stdout.flush()

    def process_key_token(self, key_token: str) -> bool:
        """Handle a parsed key token and update motion or watchdog state.

        Args:
            key_token: Normalized key identifier returned by `read_key_token`.

        Returns:
            False if the user requested termination, True to continue running.
        """
        now_sec: float = self._now_seconds()

        if key_token in QUIT_KEYS:
            self._active_direction = STOP_DIRECTION
            self.publish_direction(STOP_DIRECTION)
            return False

        if key_token in STOP_KEYS:
            self._active_direction = STOP_DIRECTION
            self._last_command_time_sec = now_sec
            self.publish_direction(STOP_DIRECTION)
            return True

        if key_token in SPEED_UP_KEYS:
            self.adjust_speeds(SPEED_SCALE_INCREMENT)
            return True

        if key_token in SPEED_DOWN_KEYS:
            self.adjust_speeds(SPEED_SCALE_DECREMENT)
            return True

        if key_token in KEY_DIRECTION_BINDINGS:
            self._active_direction = KEY_DIRECTION_BINDINGS[key_token]
            self._last_command_time_sec = now_sec
            self.publish_direction(self._active_direction)
            return True

        # Deadman watchdog: automatically stop when keys are released
        elapsed_sec: float = now_sec - self._last_command_time_sec
        if (
            self._active_direction != STOP_DIRECTION
            and elapsed_sec > DEADMAN_WATCHDOG_TIMEOUT_SEC
        ):
            self._active_direction = STOP_DIRECTION
            self.publish_direction(STOP_DIRECTION)
        elif self._active_direction != STOP_DIRECTION:
            self.publish_direction(self._active_direction)

        return True


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the `keyboard_teleop` ROS 2 executable.

    Args:
        args: Optional command-line arguments forwarded to `rclpy.init`.
    """
    if not sys.stdin.isatty():
        sys.stderr.write(
            "Error: keyboard_teleop requires an interactive terminal (TTY).\n"
            "Run it directly in a terminal with: ros2 run robot1_bringup keyboard_teleop\n"
        )
        sys.exit(1)

    stdin_fd: int = sys.stdin.fileno()
    original_term_settings = termios.tcgetattr(stdin_fd)

    rclpy.init(args=args)
    teleop_node = KeyboardTeleopNode()

    try:
        sys.stdout.write(HELP_BANNER)
        teleop_node.print_status_line()

        keep_running: bool = True
        while rclpy.ok() and keep_running:
            token: str = read_key_token(stdin_fd, original_term_settings)
            keep_running = teleop_node.process_key_token(token)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.publish_direction(STOP_DIRECTION)
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_term_settings)
        teleop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.stdout.write("\nRobot stopped. Exiting keyboard teleoperation.\n")


if __name__ == "__main__":
    main()

