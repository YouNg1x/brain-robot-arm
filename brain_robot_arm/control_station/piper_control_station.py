#!/usr/bin/env python3
"""PiPER 多功能机械臂控制台（Tkinter，无额外 GUI 依赖）。"""

from __future__ import annotations

import queue
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from datetime import datetime
from tkinter import messagebox, ttk

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    C_PiperInterface_V2 = None


BG = "#050b14"
PANEL = "#0b1726"
PANEL_2 = "#10243a"
CYAN = "#23d5ff"
GREEN = "#36f1a1"
AMBER = "#ffbd4a"
RED = "#ff416c"
TEXT = "#d9f3ff"
MUTED = "#7895aa"

RAD_TO_MDEG = 57295.7795


@dataclass(frozen=True)
class Limits:
    # 保守的软件工作空间；实机应按安装环境进一步收紧。
    x: tuple[int, int] = (50, 600)
    y: tuple[int, int] = (-450, 450)
    z: tuple[int, int] = (50, 650)
    angle: tuple[int, int] = (-180000, 180000)
    speed: tuple[int, int] = (1, 50)
    gripper: tuple[int, int] = (0, 70000)


class ArmWorker(threading.Thread):
    """串行执行机械臂命令，避免多个 GUI 回调并发写 CAN。"""

    def __init__(self, events: queue.Queue):
        super().__init__(daemon=True)
        self.events = events
        self.commands: queue.Queue = queue.Queue()
        self.arm = None
        self.connected = False
        self.enabled = False
        self.abort = threading.Event()

    def emit(self, kind: str, payload):
        self.events.put((kind, payload))

    def submit(self, name: str, *args):
        self.commands.put((name, args))

    def run(self):
        while True:
            name, args = self.commands.get()
            try:
                if name == "shutdown":
                    return
                getattr(self, f"do_{name}")(*args)
            except Exception as exc:
                self.emit("error", f"{name}: {exc}")

    def require_arm(self):
        if not self.connected or self.arm is None:
            raise RuntimeError("尚未连接机械臂")

    def require_enabled(self):
        self.require_arm()
        if not self.enabled:
            raise RuntimeError("机械臂尚未使能")

    def do_connect(self):
        if C_PiperInterface_V2 is None:
            raise RuntimeError("未找到 piper_sdk，请先 conda activate base")
        if self.connected:
            self.emit("log", "CAN0 已连接")
            return
        self.arm = C_PiperInterface_V2("can0")
        self.arm.ConnectPort()
        self.connected = True
        self.emit("state", (True, self.enabled))
        self.emit("log", "CAN0 连接建立")

    def do_enable(self):
        self.require_arm()
        self.abort.clear()
        self.arm.EnableArm(1)
        time.sleep(0.8)
        self.arm.EnableArm(1)
        self.enabled = True
        self.emit("state", (True, True))
        self.emit("log", "机械臂已使能")

    def do_disable(self):
        self.require_arm()
        self.abort.set()
        self.arm.EnableArm(0)
        self.enabled = False
        self.emit("state", (True, False))
        self.emit("log", "机械臂已失能")

    def do_stop(self):
        self.abort.set()
        # 清除尚未执行的运动任务。
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        if self.arm is not None:
            self.arm.MotionCtrl_2(0x00, 0x00, 0, 0x00)
            self.arm.EnableArm(0)
        self.enabled = False
        self.emit("state", (self.connected, False))
        self.emit("log", "紧急停止：运动停止并失能")

    def do_reset(self):
        self.require_arm()
        self.arm.MotionCtrl_1(0x01, 0, 0)
        time.sleep(0.5)
        self.arm.MotionCtrl_1(0x02, 0, 0)
        time.sleep(0.5)
        self.arm.MotionCtrl_1(0x00, 0, 0)
        self.emit("log", "控制模式复位完成")

    def do_joint(self, joints_deg: list[float], speed: int):
        self.require_enabled()
        values = [round(v * 1000) for v in joints_deg]
        self.arm.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        self.arm.JointCtrl(*values)
        self.emit("log", f"关节目标已发送：{joints_deg}°，速度 {speed}")

    def do_pose(self, pose: list[int], speed: int):
        self.require_enabled()
        self.arm.MotionCtrl_2(0x01, 0x00, speed, 0x00)
        self.arm.EndPoseCtrl(*pose)
        self.emit("log", f"末端目标已发送：{pose}，速度 {speed}")

    def do_gripper(self, position: int):
        self.require_enabled()
        self.arm.GripperCtrl(position, 1000, 0x01, 0)
        self.emit("log", f"夹爪目标：{position}")

    def do_sequence(self, points: list[list[int]], speed: int, dwell: float):
        self.require_enabled()
        self.abort.clear()
        self.arm.MotionCtrl_2(0x01, 0x00, speed, 0x00)
        for index, point in enumerate(points, 1):
            if self.abort.is_set():
                self.emit("log", "预设轨迹已中止")
                return
            self.arm.EndPoseCtrl(*point)
            self.emit("log", f"巡航点 {index}/{len(points)}：{point}")
            end = time.monotonic() + dwell
            while time.monotonic() < end:
                if self.abort.wait(0.05):
                    return
        self.emit("log", "预设轨迹完成")


class ControlStation(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("PIPER // AEROSPACE CONTROL STATION")
        self.geometry("1280x780")
        self.minsize(1080, 680)
        self.configure(bg=BG)
        self.limits = Limits()
        self.events: queue.Queue = queue.Queue()
        self.worker = ArmWorker(self.events)
        self.worker.start()
        self.connected = False
        self.enabled = False
        self._configure_style()
        self._build()
        self.after(100, self._poll_events)
        self.after(500, self._clock)
        self.protocol("WM_DELETE_WINDOW", self._close)

    def _configure_style(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", background=PANEL, foreground=MUTED,
                        padding=(22, 10), font=("Segoe UI", 10, "bold"))
        style.map("TNotebook.Tab", background=[("selected", PANEL_2)],
                  foreground=[("selected", CYAN)])
        style.configure("TScale", background=PANEL)

    def _build(self):
        header = tk.Frame(self, bg=BG, height=78)
        header.pack(fill="x", padx=22, pady=(16, 8))
        tk.Label(header, text="PIPER // CONTROL STATION", bg=BG, fg=TEXT,
                 font=("Segoe UI", 22, "bold")).pack(side="left")
        tk.Label(header, text="  MULTI-AXIS ROBOTIC OPERATIONS", bg=BG, fg=CYAN,
                 font=("Consolas", 10)).pack(side="left", pady=(12, 0))
        self.clock_label = tk.Label(header, bg=BG, fg=MUTED, font=("Consolas", 11))
        self.clock_label.pack(side="right")

        status = tk.Frame(self, bg=PANEL, highlightbackground="#17324c", highlightthickness=1)
        status.pack(fill="x", padx=22, pady=(0, 12))
        self.link_label = self._status_item(status, "CAN LINK", "OFFLINE", RED)
        self.power_label = self._status_item(status, "ARM POWER", "DISABLED", AMBER)
        self.mode_label = self._status_item(status, "CONTROL", "STANDBY", MUTED)
        self._button(status, "连接 CAN0", lambda: self.worker.submit("connect"), CYAN).pack(side="right", padx=8, pady=12)
        self._button(status, "使能", self._enable_confirm, GREEN).pack(side="right", padx=8, pady=12)
        self._button(status, "失能", lambda: self.worker.submit("disable"), AMBER).pack(side="right", padx=8, pady=12)
        self._button(status, "紧急停止", self._emergency, RED, width=14).pack(side="right", padx=14, pady=8)

        body = tk.PanedWindow(self, orient="horizontal", bg=BG, sashwidth=6,
                             sashrelief="flat", showhandle=False)
        body.pack(fill="both", expand=True, padx=22, pady=(0, 18))
        controls = tk.Frame(body, bg=PANEL, highlightbackground="#17324c", highlightthickness=1)
        console = tk.Frame(body, bg=PANEL, highlightbackground="#17324c", highlightthickness=1)
        body.add(controls, minsize=720, stretch="always")
        body.add(console, minsize=300)

        tabs = ttk.Notebook(controls)
        tabs.pack(fill="both", expand=True, padx=8, pady=8)
        self._joint_tab(tabs)
        self._pose_tab(tabs)
        self._mission_tab(tabs)
        self._console(console)

    def _status_item(self, parent, title, value, color):
        frame = tk.Frame(parent, bg=PANEL)
        frame.pack(side="left", padx=20, pady=10)
        tk.Label(frame, text=title, bg=PANEL, fg=MUTED, font=("Consolas", 8)).pack(anchor="w")
        label = tk.Label(frame, text=f"● {value}", bg=PANEL, fg=color,
                         font=("Consolas", 11, "bold"))
        label.pack(anchor="w")
        return label

    def _button(self, parent, text, command, color, width=11):
        return tk.Button(parent, text=text, command=command, bg=PANEL_2, fg=color,
                         activebackground=color, activeforeground=BG, relief="flat",
                         bd=0, width=width, pady=7, cursor="hand2",
                         font=("Segoe UI", 10, "bold"))

    def _panel_title(self, parent, title, subtitle):
        tk.Label(parent, text=title, bg=PANEL, fg=TEXT,
                 font=("Segoe UI", 17, "bold")).pack(anchor="w", padx=22, pady=(20, 2))
        tk.Label(parent, text=subtitle, bg=PANEL, fg=MUTED,
                 font=("Segoe UI", 9)).pack(anchor="w", padx=22, pady=(0, 18))

    def _joint_tab(self, tabs):
        tab = tk.Frame(tabs, bg=PANEL)
        tabs.add(tab, text="关节控制")
        self._panel_title(tab, "六轴关节控制", "输入角度单位为度；首次实机测试请使用低速和小角度")
        grid = tk.Frame(tab, bg=PANEL)
        grid.pack(fill="x", padx=22)
        self.joint_vars = []
        defaults = [0, -17.2, 0, 0, 0, 0]
        for i in range(6):
            card = tk.Frame(grid, bg=PANEL_2)
            card.grid(row=i // 3, column=i % 3, padx=7, pady=7, sticky="ew")
            grid.columnconfigure(i % 3, weight=1)
            tk.Label(card, text=f"JOINT {i + 1}", bg=PANEL_2, fg=CYAN,
                     font=("Consolas", 10, "bold")).pack(pady=(10, 3))
            var = tk.StringVar(value=str(defaults[i]))
            self.joint_vars.append(var)
            tk.Entry(card, textvariable=var, bg=BG, fg=TEXT, insertbackground=CYAN,
                     justify="center", relief="flat", font=("Consolas", 15), width=12).pack(padx=15, pady=(0, 12))
        row = tk.Frame(tab, bg=PANEL)
        row.pack(fill="x", padx=28, pady=22)
        self.joint_speed = self._speed(row)
        self._button(row, "发送关节目标", self._send_joints, GREEN, 18).pack(side="right")

    def _pose_tab(self, tabs):
        tab = tk.Frame(tabs, bg=PANEL)
        tabs.add(tab, text="末端位姿")
        self._panel_title(tab, "末端位姿控制", "位置单位 mm，姿态单位 0.001°；软件限位仅是附加保护")
        grid = tk.Frame(tab, bg=PANEL)
        grid.pack(fill="x", padx=22)
        self.pose_vars = []
        for i, (name, default) in enumerate(zip(("X", "Y", "Z", "RX", "RY", "RZ"), (157, 0, 215, 0, 85000, 0))):
            card = tk.Frame(grid, bg=PANEL_2)
            card.grid(row=i // 3, column=i % 3, padx=7, pady=7, sticky="ew")
            grid.columnconfigure(i % 3, weight=1)
            tk.Label(card, text=name, bg=PANEL_2, fg=CYAN,
                     font=("Consolas", 10, "bold")).pack(pady=(10, 3))
            var = tk.StringVar(value=str(default))
            self.pose_vars.append(var)
            tk.Entry(card, textvariable=var, bg=BG, fg=TEXT, insertbackground=CYAN,
                     justify="center", relief="flat", font=("Consolas", 15), width=12).pack(padx=15, pady=(0, 12))
        row = tk.Frame(tab, bg=PANEL)
        row.pack(fill="x", padx=28, pady=18)
        self.pose_speed = self._speed(row)
        self._button(row, "发送末端目标", self._send_pose, GREEN, 18).pack(side="right")
        grip = tk.Frame(tab, bg=PANEL_2)
        grip.pack(fill="x", padx=28, pady=8)
        tk.Label(grip, text="GRIPPER", bg=PANEL_2, fg=TEXT,
                 font=("Consolas", 10, "bold")).pack(side="left", padx=16)
        self.gripper = tk.Scale(grip, from_=0, to=70000, orient="horizontal", bg=PANEL_2,
                                fg=TEXT, troughcolor=BG, highlightthickness=0, length=300)
        self.gripper.set(30000)
        self.gripper.pack(side="left", fill="x", expand=True, pady=8)
        self._button(grip, "发送夹爪", self._send_gripper, CYAN).pack(side="right", padx=12)

    def _mission_tab(self, tabs):
        tab = tk.Frame(tabs, bg=PANEL)
        tabs.add(tab, text="任务中心")
        self._panel_title(tab, "任务与预设动作", "预设点来自原 move_to_points.py；首次运行必须有人监护")
        cards = tk.Frame(tab, bg=PANEL)
        cards.pack(fill="x", padx=22)
        actions = [
            ("模式复位", "主/从模式切换并复位控制状态", lambda: self.worker.submit("reset"), AMBER),
            ("安全参考位", "移动至 X57 Y0 Z215 RY85°", self._safe_pose, GREEN),
            ("XYZ 巡航演示", "依次测试 X/Y/Z 方向点位", self._patrol, CYAN),
            ("夹爪张开", "夹爪目标位置 50000", lambda: self.worker.submit("gripper", 50000), CYAN),
            ("夹爪闭合", "夹爪目标位置 1000", lambda: self.worker.submit("gripper", 1000), AMBER),
        ]
        for i, (title, desc, cmd, color) in enumerate(actions):
            card = tk.Frame(cards, bg=PANEL_2)
            card.grid(row=i // 2, column=i % 2, sticky="ew", padx=7, pady=7)
            cards.columnconfigure(i % 2, weight=1)
            tk.Label(card, text=title, bg=PANEL_2, fg=color,
                     font=("Segoe UI", 12, "bold")).pack(anchor="w", padx=14, pady=(12, 2))
            tk.Label(card, text=desc, bg=PANEL_2, fg=MUTED,
                     font=("Segoe UI", 9)).pack(anchor="w", padx=14)
            self._button(card, "执行", cmd, color).pack(anchor="e", padx=12, pady=10)

    def _speed(self, parent):
        tk.Label(parent, text="SPEED", bg=PANEL, fg=MUTED,
                 font=("Consolas", 9, "bold")).pack(side="left")
        scale = tk.Scale(parent, from_=1, to=50, orient="horizontal", bg=PANEL,
                         fg=TEXT, troughcolor=BG, highlightthickness=0, length=250)
        scale.set(20)
        scale.pack(side="left", padx=10)
        return scale

    def _console(self, parent):
        tk.Label(parent, text="MISSION LOG", bg=PANEL, fg=CYAN,
                 font=("Consolas", 12, "bold")).pack(anchor="w", padx=15, pady=(16, 8))
        self.log_box = tk.Text(parent, bg="#030810", fg=GREEN, insertbackground=CYAN,
                               relief="flat", state="disabled", wrap="word",
                               font=("Consolas", 9), padx=10, pady=10)
        self.log_box.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self._log("控制台启动；请先初始化 CAN，再连接 CAN0")

    def _enable_confirm(self):
        if messagebox.askyesno("使能确认", "确认机械臂周围无人、无障碍物，并且急停可用？"):
            self.worker.submit("enable")

    def _emergency(self):
        # 直接设置中止标志，同时把停止指令放到队列头部无法保证；worker 的循环会迅速感知。
        self.worker.abort.set()
        self.worker.submit("stop")
        self._log("紧急停止请求已发送")

    def _send_joints(self):
        try:
            values = [float(v.get()) for v in self.joint_vars]
            if any(not -180 <= v <= 180 for v in values):
                raise ValueError("关节输入必须在 -180° 至 180° 的软件范围内")
            self.worker.submit("joint", values, int(self.joint_speed.get()))
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))

    def _send_pose(self):
        try:
            pose = [int(v.get()) for v in self.pose_vars]
            ranges = (self.limits.x, self.limits.y, self.limits.z,
                      self.limits.angle, self.limits.angle, self.limits.angle)
            for name, value, bounds in zip(("X", "Y", "Z", "RX", "RY", "RZ"), pose, ranges):
                if not bounds[0] <= value <= bounds[1]:
                    raise ValueError(f"{name}={value} 超出软件范围 {bounds}")
            self.worker.submit("pose", pose, int(self.pose_speed.get()))
        except ValueError as exc:
            messagebox.showerror("输入错误", str(exc))

    def _send_gripper(self):
        self.worker.submit("gripper", int(self.gripper.get()))

    def _safe_pose(self):
        self.worker.submit("pose", [57, 0, 215, 0, 85000, 0], 15)

    def _patrol(self):
        if not messagebox.askyesno("巡航确认", "即将执行多个 XYZ 点位，确认工作空间安全？"):
            return
        points = [[57, 0, 215, 0, 85000, 0], [157, 0, 215, 0, 85000, 0],
                  [57, 100, 215, 0, 85000, 0], [57, 0, 315, 0, 85000, 0],
                  [57, 0, 215, 0, 85000, 0]]
        self.worker.submit("sequence", points, 15, 3.0)

    def _poll_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    self._log(payload)
                elif kind == "error":
                    self._log(f"ERROR // {payload}")
                    messagebox.showerror("控制错误", payload)
                elif kind == "state":
                    self.connected, self.enabled = payload
                    self.link_label.config(text=f"● {'ONLINE' if self.connected else 'OFFLINE'}",
                                           fg=GREEN if self.connected else RED)
                    self.power_label.config(text=f"● {'ENABLED' if self.enabled else 'DISABLED'}",
                                            fg=GREEN if self.enabled else AMBER)
                    self.mode_label.config(text=f"● {'ACTIVE' if self.enabled else 'STANDBY'}",
                                           fg=CYAN if self.enabled else MUTED)
        except queue.Empty:
            pass
        self.after(100, self._poll_events)

    def _log(self, message: str):
        stamp = datetime.now().strftime("%H:%M:%S")
        self.log_box.configure(state="normal")
        self.log_box.insert("end", f"[{stamp}] {message}\n")
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    def _clock(self):
        self.clock_label.config(text=datetime.now().strftime("%Y-%m-%d // %H:%M:%S"))
        self.after(500, self._clock)

    def _close(self):
        if self.enabled and not messagebox.askyesno("退出确认", "机械臂仍处于使能状态。确定退出控制台？"):
            return
        self.worker.submit("shutdown")
        self.destroy()


if __name__ == "__main__":
    ControlStation().mainloop()
