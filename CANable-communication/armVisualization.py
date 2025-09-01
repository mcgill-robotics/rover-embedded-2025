import tkinter as tk
import math
import json
import os

SHOULDER_LEN = 120
ELBOW_LEN = 80

# Offsets so that "zero" means mechanical home
SHOULDER_ZERO_OFFSET = 45    # Shoulder zero is at 45 degrees up from vertical
ELBOW_ZERO_OFFSET = -135     # Elbow zero is -135 deg relative to shoulder

class ArmVisualizer(tk.Tk):
    def __init__(self, angles_file="arm_angles.json"):
        super().__init__()
        self.title("2-Joint Arm Visualizer")
        self.geometry("500x400")
        self.canvas = tk.Canvas(self, width=400, height=300, bg="white")
        self.canvas.pack(pady=10)

        self.angles_file = angles_file
        self.shoulder = 0
        self.elbow = 0

        self.after(100, self.update_from_file)

    def update_from_file(self):
        if os.path.exists(self.angles_file):
            try:
                with open(self.angles_file, "r") as f:
                    data = json.load(f)
                    self.shoulder = data.get("shoulder", 0)
                    self.elbow = data.get("elbow", 0)
            except Exception as e:
                pass  # optionally log

        self.draw_arm()
        self.after(100, self.update_from_file)

    def draw_arm(self):
        self.canvas.delete("all")
        # Apply zero offsets
        shoulder_angle = math.radians(self.shoulder + SHOULDER_ZERO_OFFSET)
        elbow_angle = math.radians(self.elbow + ELBOW_ZERO_OFFSET)

        # Origin/base of the shoulder
        x0, y0 = 200, 250

        # First segment (shoulder)
        x1 = x0 + SHOULDER_LEN * math.sin(shoulder_angle)
        y1 = y0 - SHOULDER_LEN * math.cos(shoulder_angle)
        self.canvas.create_line(x0, y0, x1, y1, width=4, fill="blue")

        # Second segment (elbow)
        angle2 = shoulder_angle + elbow_angle
        x2 = x1 + ELBOW_LEN * math.sin(angle2)
        y2 = y1 - ELBOW_LEN * math.cos(angle2)
        self.canvas.create_line(x1, y1, x2, y2, width=4, fill="red")

        # Draw joints
        for (x, y) in [(x0, y0), (x1, y1), (x2, y2)]:
            self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="orange")

if __name__ == "__main__":
    ArmVisualizer().mainloop()
