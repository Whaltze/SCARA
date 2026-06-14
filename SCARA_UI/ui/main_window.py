import numpy as np
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QMainWindow

from ..communication.serial_mixin import ScaraSerialMixin
from ..core.feedback_error import FeedbackErrorTracker
from ..core.kinematics import FiveBarConfig, FiveBarKinematics
from ..core.utility_mixin import ScaraUtilityMixin
from ..motion.motion_mixin import ScaraMotionMixin
from ..trajectory.look_ahead import LookAheadPlanner
from ..vision.camera_core import CameraProcessor
from ..vision.coordinate_core import CoordinateProcessor
from ..vision.threads import ImageProcessingThread
from ..vision.vision_mixin import ScaraVisionMixin
from .plotting import ScaraPlotMixin
from .ui_mixin import ScaraUiMixin


class FiveBarSerialGUI(
    ScaraUiMixin,
    ScaraUtilityMixin,
    ScaraVisionMixin,
    ScaraSerialMixin,
    ScaraMotionMixin,
    ScaraPlotMixin,
    QMainWindow,
):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SCARA Serial Control System")
        self.resize(1220, 820)

        self.ser = None
        self.L0, self.L1, self.L2 = 150.0, 160.0, 200.0
        self.A, self.B = np.array([0, 0]), np.array([self.L0, 0])
        self.HOME_X, self.HOME_Y = 75.0, 220.0

        self.accel = 100.0

        self.junction_dev = 0.02
        
        self.dt = 0.02

        self.kinematics = FiveBarKinematics(
            FiveBarConfig(
                base_distance=self.L0,
                active_link=self.L1,
                passive_link=self.L2,
            )
        )
        self.path_planner = LookAheadPlanner(
            accel_mm_s2=self.accel,
            junction_deviation=self.junction_dev,
            sample_dt=self.dt,
            max_segment_mm=0.35,
            min_segment_mm=0.02,
        )

        self.cam_proc = CameraProcessor()
        self.coord_proc = CoordinateProcessor()

        home_90 = self.kinematics.forward(90.0, 90.0)
        if home_90[0] is not None and home_90[1] is not None:
            self.HOME_X, self.HOME_Y = float(home_90[0]), float(home_90[1])
        else:
            self.HOME_X, self.HOME_Y = self.kinematics.find_safe_home((self.HOME_X, self.HOME_Y))
        self.cur_x, self.cur_y = self.HOME_X, self.HOME_Y
        self.history_x, self.history_y = [self.cur_x], [self.cur_y]
        self.feedback_x, self.feedback_y = [], []
        self.preview_x, self.preview_y = [], []
        self.preview_label = ""
        self.feedback_error_tracker = FeedbackErrorTracker()
        self.latest_feedback_error = None
        self.feedback_error_stats = None
        self._plot_user_view = False
        self.velocity_monitor = None
        self.is_silent_move = False
        self.is_recording = False
        self.teach_data = []
        self.teach_points = []
        self.teach_step_index = 0
        self.point_queue = []
        self.current_ppr = 6400
        self.microstep_dirty = True

        self.waiting_for_ack = False
        self.last_sent_motion = None
        self.sent_point_id = 0
        self.total_task_points = 0
        self.task_start_time = 0
        self.error_count = 0
        self.heartbeat_count = 0
        self.ack_timeout_count = 0
        self.idle_ack_stall_polls = 0
        self.last_controller_rx_at = 0.0
        self.last_controller_state = ""
        self.last_segment_count = 0
        # Cumulative MCU diagnostics establish a baseline on first status.
        self.planner_fault_count = None
        self.rate_limited_segment_count = 0
        self.serial_failure_reported = False
        self.mcu_planner_capacity = 48
        self.mcu_planner_free = self.mcu_planner_capacity
        self.planner_free_hint = self.mcu_planner_capacity
        self.planner_free_min = self.mcu_planner_capacity
        self.rx_free_hint = 256
        self.stream_waiting_buffer = False
        self.motion_preamble_needed = True
        self.motion_profile_sync_requested = False
        self.laser_task_active = False
        self.laser_preamble_needed = False
        self.laser_power_permille = 10
        self.laser_arm_sent_at = 0.0
        self.pending_laser_power_permille = None
        self.controller_capabilities = None
        self.controller_reset_pending = False
        self.controller_reset_reason = ""
        self.controller_reset_started_at = 0.0
        self.controller_reset_generation = 0
        self.emergency_paused = False
        self.active_preview_path = []
        self.jog_target_xy = None

        # 如果接了真实电机和 HOME 开关，应该改成：
        self.board_only_debug = False
        self.is_homed = False

        # self.board_only_debug = True
        # self.is_homed = self.board_only_debug


        self.home_sensor_triggered = False

        self.move_timer = QTimer()
        self.read_timer = QTimer()
        self.read_timer.timeout.connect(self.check_serial_feedback)
        self.read_timer.start(2)

        self.timeout_timer = QTimer()
        self.timeout_timer.setSingleShot(True)
        self.timeout_timer.timeout.connect(self.handle_timeout)

        self.heartbeat_timer = QTimer()
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)

        self.latest_raw_frame = None
        self.cam_thread = None
        self.img_proc_thread = ImageProcessingThread(self.cam_proc)
        self.img_proc_thread.processed_frame_ready.connect(self.display_camera_frame)
        self.img_proc_thread.start()

        self.init_ui()
        self.precompute_workspace()
        self.refresh_ports()
        self.update_plot()
        self.update_v_params()
        self.log_display.append(
            "<font color='#cccccc'>>>> Vision module loaded, ready to coordinate with SCARA_F103.</font>"
        )
