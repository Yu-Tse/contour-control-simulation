
````markdown
# 🧪 Contour-Control Simulation (State-Space Based)

This repository contains a simulation of a 2-axis contour control system, completed as a final assignment for the **Numerical Methods** course (Lab 6 – Dec 2022). The system uses a state-space model to track a predefined geometric path composed of lines and arcs, under different feedrates.

> ✅ This project was written in Python with assistance from ChatGPT for code structuring and debugging. All logic and final implementation were verified and tested by the author.

---

## 📌 Objectives

- Convert a given transfer function into a **state-space** model.
- Construct a **2D geometric path** made of lines and arcs.
- Implement a **contour control system** with outer P-loop and inner PI velocity control.
- Simulate the system using **discrete-time control** with sampling period `Ts = 0.5 ms`.
- Compare the **actual path vs. reference path**, and plot **contour errors** under varying constant feedrates.

---

## 🛠️ Features

- ✅ Path construction using geometric primitives (`line`, `arc`)
- ✅ Feedrate interpolation and velocity reference generation
- ✅ State-space modeling and discrete-time simulation using `control` package
- ✅ Closed-loop contour-following simulation (with feedforward)
- ✅ Trajectory plotting and contour error analysis
- ✅ Supports feedrates: `200`, `500`, `800 mm/min`

---

## 📁 File Structure

```bash
contour-control-sim/
├── contour_sim.py      # Main Python script
├── README.md           # This file
└── figures/
          └── contour error.png
              results.png

````

---

## 📊 Simulation Results

Contour following was tested at three different constant feedrates. The RMSE and maximum contour errors are printed during execution.

```text

V=200 mm/min  RMSE= 8.11 µm   Max=29.15 µm
V=500 mm/min  RMSE=20.43 µm   Max=69.92 µm
V=800 mm/min  RMSE=35.01 µm   Max=117.33 µm

```

The output includes:

* Trajectory comparison plot (reference vs. actual)
* Contour error plot over time

---

## 🧪 Dependencies

Install the required packages using:

```bash
pip install numpy matplotlib control
```

---

## 🧑‍🎓 Author

**Yu-Tse Wu (吳雨澤)**
National Chung Cheng University
Department of Mechanical Engineering

---

## 📘 Notes

* This project was completed as a university lab assignment.
* Some code logic was enhanced using AI-assisted suggestions (ChatGPT).
* For educational and portfolio purposes only.
* Please do not use this for plagiarism or without attribution.

---

## 🪪 License

MIT License – feel free to use for learning or non-commercial use.
