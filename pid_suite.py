"""
Advanced PID Control & System Analysis Suite
Provides real-time simulation, automated PID tuning (via SciPy), 
and frequency/time domain stability analysis (Bode, Pole-Zero, Step Response).
"""

import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import control as ct
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.optimize import minimize

# Apply dark theme to matplotlib
plt.style.use('dark_background')

class AdvancedControlSuite:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Control Systems Suite | Auto-Tuning & Analysis")
        self.root.geometry("1400x850")
        self.root.configure(bg="#1e1e1e")
        
        self.colors = {
            "bg": "#1e1e1e",
            "panel": "#252526",
            "text": "#cccccc",
            "accent": "#007acc",
            "warning": "#e51400",
            "success": "#339933"
        }
        
        self.setup_styles()
        self.build_ui()
        
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TFrame", background=self.colors["panel"])
        style.configure("TLabel", background=self.colors["panel"], foreground=self.colors["text"], font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"), foreground="#ffffff")
        style.configure("TButton", background=self.colors["accent"], foreground="#ffffff", font=("Segoe UI", 10, "bold"), borderwidth=0)
        style.map("TButton", background=[("active", "#005f9e")])
        style.configure("TEntry", fieldbackground="#3c3c3c", foreground="#ffffff", borderwidth=1)

    def build_ui(self):
        # --- Left Panel (Inputs) ---
        sidebar = ttk.Frame(self.root, padding=20)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(sidebar, text="PLANT DYNAMICS G(s)", style="Header.TLabel").pack(pady=(0, 15), anchor="w")
        
        self.num_entry = self.create_input_group(sidebar, "Numerator (e.g., 1)")
        self.num_entry.insert(0, "1")
        
        self.den_entry = self.create_input_group(sidebar, "Denominator (e.g., 1, 3, 2)")
        self.den_entry.insert(0, "1, 3, 2")

        ttk.Label(sidebar, text="\nPID CONTROLLER GAINS", style="Header.TLabel").pack(pady=(15, 15), anchor="w")
        
        self.kp_entry = self.create_input_group(sidebar, "Proportional (Kp)")
        self.kp_entry.insert(0, "5.0")
        
        self.ki_entry = self.create_input_group(sidebar, "Integral (Ki)")
        self.ki_entry.insert(0, "1.0")
        
        self.kd_entry = self.create_input_group(sidebar, "Derivative (Kd)")
        self.kd_entry.insert(0, "0.5")

        ttk.Label(sidebar, text="\nSIMULATION PARAMS", style="Header.TLabel").pack(pady=(15, 15), anchor="w")
        self.setpoint_entry = self.create_input_group(sidebar, "Setpoint (Target)")
        self.setpoint_entry.insert(0, "1.0")

        # Buttons
        btn_frame = ttk.Frame(sidebar)
        btn_frame.pack(fill=tk.X, pady=25)
        
        ttk.Button(btn_frame, text="▶ RUN ANALYSIS", command=self.run_analysis).pack(fill=tk.X, pady=5, ipady=5)
        
        opt_btn = ttk.Button(btn_frame, text="⚡ AUTO-TUNE PID", command=self.auto_tune)
        opt_btn.pack(fill=tk.X, pady=5, ipady=5)
        opt_btn.configure(style="TButton") 

        # Metrics display
        self.metrics_text = tk.Text(sidebar, height=8, width=35, bg="#1e1e1e", fg="#00ff99", font=("Consolas", 10), bd=0)
        self.metrics_text.pack(pady=10)
        self.metrics_text.insert(tk.END, "System Ready...\nAwaiting analysis.")
        self.metrics_text.config(state="disabled")

        # --- Right Panel (Dashboard) ---
        self.graph_frame = tk.Frame(self.root, bg=self.colors["bg"])
        self.graph_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8), facecolor=self.colors["bg"])
        self.fig.tight_layout(pad=4.0)
        
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def create_input_group(self, parent, label_text):
        ttk.Label(parent, text=label_text).pack(anchor="w")
        entry = ttk.Entry(parent, font=("Consolas", 11))
        entry.pack(fill=tk.X, pady=(2, 10))
        return entry

    def get_system_model(self, kp, ki, kd):
        num = [float(x.strip()) for x in self.num_entry.get().split(",")]
        den = [float(x.strip()) for x in self.den_entry.get().split(",")]
        
        plant = ct.tf(num, den)
        pid = ct.tf([kd, kp, ki], [1, 0])
        closed_loop = ct.feedback(pid * plant)
        
        return closed_loop

    def update_metrics(self, y, t, setpoint):
        overshoot = 0
        if np.max(y) > setpoint:
            overshoot = (np.max(y) - setpoint) / setpoint * 100
            
        sse = abs(setpoint - y[-1])
        ise = np.sum((setpoint - y)**2) * (t[1] - t[0])
        
        self.metrics_text.config(state="normal")
        self.metrics_text.delete(1.0, tk.END)
        report = (
            f"--- PERFORMANCE METRICS ---\n\n"
            f"Overshoot   : {overshoot:.2f} %\n"
            f"Steady Error: {sse:.4f}\n"
            f"ISE (Cost)  : {ise:.4f}\n\n"
            f"Status      : {'STABLE' if overshoot < 25 else 'WARNING/UNSTABLE'}"
        )
        self.metrics_text.insert(tk.END, report)
        self.metrics_text.config(state="disabled")
        
        return ise, overshoot

    def run_analysis(self, update_gui=True):
        try:
            kp = float(self.kp_entry.get())
            ki = float(self.ki_entry.get())
            kd = float(self.kd_entry.get())
            setpoint = float(self.setpoint_entry.get())

            sys_cl = self.get_system_model(kp, ki, kd)
            
            # 1. Step Response Data
            t, y = ct.step_response(sys_cl, T=np.linspace(0, 15, 1000))
            y = y * setpoint
            
            ise, overshoot = self.update_metrics(y, t, setpoint)

            if update_gui:
                for ax in self.axs.flat:
                    ax.clear()
                    ax.set_facecolor('#121212')
                    ax.grid(True, color='#333333', linestyle='--')

                # Plot 1: Step Response
                self.axs[0, 0].plot(t, y, color="#00ff99", linewidth=2)
                self.axs[0, 0].axhline(setpoint, color="#ff3366", linestyle="--", alpha=0.7)
                self.axs[0, 0].set_title("Closed-Loop Step Response", color="#ffffff")
                self.axs[0, 0].set_xlabel("Time (s)")
                self.axs[0, 0].set_ylabel("Amplitude")

                # Plot 2: Bode Plot (Magnitude)
                mag, phase, omega = ct.bode(sys_cl, plot=False)
                self.axs[0, 1].semilogx(omega, 20 * np.log10(mag), color="#00aaff", linewidth=2)
                self.axs[0, 1].set_title("Bode Plot (Magnitude)", color="#ffffff")
                self.axs[0, 1].set_xlabel("Frequency (rad/s)")
                self.axs[0, 1].set_ylabel("Magnitude (dB)")

                # Plot 3: Pole-Zero Map
                poles, zeros = ct.pzmap(sys_cl, plot=False)
                self.axs[1, 0].axvline(0, color='gray', linewidth=1)
                self.axs[1, 0].axhline(0, color='gray', linewidth=1)
                self.axs[1, 0].scatter(np.real(poles), np.imag(poles), marker='x', color='#ff3366', s=100, label='Poles')
                if len(zeros) > 0:
                    self.axs[1, 0].scatter(np.real(zeros), np.imag(zeros), marker='o', facecolors='none', edgecolors='#00ff99', s=100, label='Zeros')
                self.axs[1, 0].set_title("Pole-Zero Map (Stability)", color="#ffffff")
                self.axs[1, 0].set_xlabel("Real Axis")
                self.axs[1, 0].set_ylabel("Imaginary Axis")
                self.axs[1, 0].legend(loc="upper left")

                # Plot 4: System Error over time
                error = setpoint - y
                self.axs[1, 1].plot(t, error, color="#ffaa00", linewidth=2)
                self.axs[1, 1].axhline(0, color="gray", linestyle="--")
                self.axs[1, 1].set_title("Tracking Error", color="#ffffff")
                self.axs[1, 1].set_xlabel("Time (s)")
                self.axs[1, 1].set_ylabel("Error")

                self.canvas.draw()

            return ise, overshoot

        except Exception as e:
            if update_gui:
                messagebox.showerror("Simulation Error", f"Failed to compute dynamics:\n{str(e)}")
            return float('inf'), float('inf')

    def auto_tune(self):
        def cost_function(pid_params):
            self.kp_entry.delete(0, tk.END); self.kp_entry.insert(0, str(pid_params[0]))
            self.ki_entry.delete(0, tk.END); self.ki_entry.insert(0, str(pid_params[1]))
            self.kd_entry.delete(0, tk.END); self.kd_entry.insert(0, str(pid_params[2]))
            
            ise, overshoot = self.run_analysis(update_gui=False)
            
            # Penalize overshoots heavily for aggressive tuning
            penalty = (overshoot * 5) if overshoot > 5.0 else 0
            return ise + penalty

        current_params = [float(self.kp_entry.get()), float(self.ki_entry.get()), float(self.kd_entry.get())]
        bounds = ((0.01, 100), (0.0, 100), (0.0, 100))
        
        self.metrics_text.config(state="normal")
        self.metrics_text.delete(1.0, tk.END)
        self.metrics_text.insert(tk.END, "Running SciPy Optimization...\nPlease wait.")
        self.metrics_text.config(state="disabled")
        self.root.update()

        result = minimize(cost_function, current_params, bounds=bounds, method='Nelder-Mead')

        self.kp_entry.delete(0, tk.END); self.kp_entry.insert(0, f"{result.x[0]:.3f}")
        self.ki_entry.delete(0, tk.END); self.ki_entry.insert(0, f"{result.x[1]:.3f}")
        self.kd_entry.delete(0, tk.END); self.kd_entry.insert(0, f"{result.x[2]:.3f}")
        
        self.run_analysis()
        messagebox.showinfo("Auto-Tune Complete", "Optimal PID gains calculated successfully.")

if __name__ == "__main__":
    root = tk.Tk()
    app = AdvancedControlSuite(root)
    root.mainloop()
