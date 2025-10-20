#!/usr/bin/env python3
"""
motor_gui.py

Tkinter GUI to control motors ID 7..13 with three modes:
- Speed mode:   publish [ID, 0, val, 0, 1, 0]    (val in 0..2)
- Position mode:publish [ID, val, 0, 1, 1, 0]    (val in 0..2)
- Torque mode:  publish [ID, 0, 0, 0, 0, val]    (val in 0..2)

Extra:
- Reset button: publish [0,0,0,0,0] to /all_joints_hjc/command_same
  to stop all motors immediately.
"""

import threading
import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Float64MultiArray

# Motor IDs
MOTOR_IDS = list(range(7, 14))  # 7..13

class MotorGUI:
    def __init__(self, master):
        self.master = master
        master.title("Motor Group Controller (IDs 7-13)")

        # ROS init (disable_signals to let Tkinter handle signals)
        rospy.init_node('motor_gui_node', anonymous=True, disable_signals=True)
        self.pub_one = rospy.Publisher('/all_joints_hjc/command_one', Float64MultiArray, queue_size=10)
        self.pub_same = rospy.Publisher('/all_joints_hjc/command_same', Float64MultiArray, queue_size=10)

        # mode: 'speed' | 'position' | 'torque'
        self.mode = tk.StringVar(value='speed')

        # UI frames
        top_frame = ttk.Frame(master, padding=8)
        top_frame.grid(row=0, column=0, sticky='ew')

        mode_frame = ttk.LabelFrame(top_frame, text="Control Mode", padding=6)
        mode_frame.grid(row=0, column=0, sticky='w', padx=4, pady=4)

        # Three buttons for modes
        ttk.Radiobutton(mode_frame, text="Speed", variable=self.mode, value='speed').grid(row=0, column=0, padx=6)
        ttk.Radiobutton(mode_frame, text="Position", variable=self.mode, value='position').grid(row=0, column=1, padx=6)
        ttk.Radiobutton(mode_frame, text="Torque", variable=self.mode, value='torque').grid(row=0, column=2, padx=6)

        # Sliders frame
        sliders_frame = ttk.LabelFrame(master, text="Sliders for Motors (IDs 7 - 13)", padding=6)
        sliders_frame.grid(row=1, column=0, sticky='ew', padx=8, pady=6)

        self.sliders = {}
        self.slider_vars = {}

        for i, mid in enumerate(MOTOR_IDS):
            row = i
            label = ttk.Label(sliders_frame, text=f"ID {mid}")
            label.grid(row=row, column=0, sticky='w', padx=4, pady=6)

            var = tk.DoubleVar(value=0.0)
            self.slider_vars[mid] = var

            s = ttk.Scale(sliders_frame, from_=0.0, to=2.0, orient='horizontal', variable=var)
            s.grid(row=row, column=1, sticky='ew', padx=4, pady=6)
            sliders_frame.columnconfigure(1, weight=1)

            val_label = ttk.Label(sliders_frame, text="0.000")
            val_label.grid(row=row, column=2, sticky='w', padx=6)

            def make_trace(lbl, v):
                def on_change(*args):
                    lbl.config(text=f"{v.get():.3f}")
                return on_change
            var.trace_add('write', make_trace(val_label, var))

            self.sliders[mid] = s

        # Controls at bottom
        bottom_frame = ttk.Frame(master, padding=8)
        bottom_frame.grid(row=2, column=0, sticky='ew')

        send_btn = ttk.Button(bottom_frame, text="Send", command=self.send_commands)
        send_btn.grid(row=0, column=0, padx=6)

        reset_btn = ttk.Button(bottom_frame, text="Reset (Stop All)", command=self.reset_motors)
        reset_btn.grid(row=0, column=1, padx=6)

        self.status_var = tk.StringVar(value="Ready")
        status_label = ttk.Label(bottom_frame, textvariable=self.status_var)
        status_label.grid(row=0, column=2, padx=12)

        # Start a small thread to keep rospy alive
        self.rospy_thread = threading.Thread(target=self.ros_spin_thread, daemon=True)
        self.rospy_thread.start()

    def ros_spin_thread(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def send_commands(self):
        mode = self.mode.get()
        published = 0
        errors = []

        for mid in MOTOR_IDS:
            val = float(self.slider_vars[mid].get())
            if val == 0.0:
                continue  # 0 不发送，避免覆盖

            if mode == 'speed':
                data = [float(mid), 0.0, float(val), 0.0, 1.0, 0.0]
            elif mode == 'position':
                data = [float(mid), float(val), 0.0, 1.0, 1.0, 0.0]
            elif mode == 'torque':
                data = [float(mid), 0.0, 0.0, 0.0, 0.0, float(val)]
            else:
                errors.append(f"Unknown mode: {mode}")
                continue

            msg = Float64MultiArray(data=data)
            try:
                self.pub_one.publish(msg)
                published += 1
                rospy.sleep(0.05)  # 避免覆盖，加个小延时
            except Exception as e:
                errors.append(f"ID {mid}: {e}")

        if errors:
            self.status_var.set(f"Published {published}. Errors: {len(errors)}")
        else:
            self.status_var.set(f"Published {published} nonzero commands ({mode} mode).")

    def reset_motors(self):
        """停止所有电机"""
        stop_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0])
        try:
            self.pub_same.publish(stop_msg)
            self.status_var.set("All motors stopped (Reset).")
            rospy.loginfo("All motors stopped (Reset).")
        except Exception as e:
            self.status_var.set(f"Reset failed: {e}")
            rospy.logerr("Reset failed: %s", e)


def main():
    root = tk.Tk()
    app = MotorGUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()
