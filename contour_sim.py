#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lab-6 ▸ Contour-control simulation
author : YOUR_NAME
"""

import numpy as np
import matplotlib.pyplot as plt
from control import ss, c2d      # pip install control
from itertools import chain
# ---------------------- 0. 公用 ----------------------
Ts   = 5e-4                        # 0.5 ms
Kb   = 4 / (2*np.pi)              # ball-screw 轉換 (mm/rad)
Vf_list = [200, 500, 800]         # mm /min

# 兩軸參數（表 1）--------------------------------------
AXES = {
    "X": dict(Kt=0.423, J=3.620e-5, B=2.860e-3,
              Kpp=156.881, Kvp=5.727e-2, Kvi=8.527, Kr=0.5),
    "Y": dict(Kt=0.413, J=3.817e-5, B=2.895e-3,
              Kpp=156.563, Kvp=6.033e-2, Kvi=9.815, Kr=0.5),
}

# ---------------------- 1. 幾何原函式 ------------------
def line(p1, p2, v, Ts):
    L     = np.linalg.norm(p2-p1)
    N     = int(np.ceil(L/v/Ts))+1
    u     = np.linspace(0, 1, N)
    traj  = (1-u)[:,None]*p1 + u[:,None]*p2
    speed = np.full(N, v)
    return traj, speed

def arc(center, r, a0, a1, v, Ts):
    L     = abs(a1-a0)*r
    N     = int(np.ceil(L/v/Ts))+1
    th    = np.linspace(a0, a1, N)
    x     = center[0] + r*np.cos(th)
    y     = center[1] + r*np.sin(th)
    traj  = np.vstack([x, y]).T
    speed = np.full(N, v)
    return traj, speed

# ---------------------- 2. 建構路徑 --------------------
SEGMENTS = [
    ('line', ((0,10),   (0,30))),          # ↑
    ('line', ((0,30),   (20,30))),         # →
    ('line', ((20,30),  (20,50))),         # ↑
    ('line', ((20,50),  (90,50))),         # →
    ('line', ((90,50),  (90,10))),         # ↓
    ('line', ((90,10),  (80,10))),         # ← 短直線
    # ↓↓↓ 三段向下凸的半圓 ↓↓↓
    ('arc', ((70,10), 10,  0, -np.pi)),    # 80 → 60
    ('line', ((50,10),(60,10))),    # 60 → 40
    ('arc', ((30,10), 10,  0, -np.pi)),    # 40 → 20
    # ↑↑↑                              ↑↑↑
    ('line', ((20,10),  (0,10))),          # ← 收尾直線
]


def build_path(vf_mm_min):
    v = vf_mm_min/60.0                # 轉 mm/s
    trajs, vels = [], []
    for typ, prm in SEGMENTS:
        if typ == 'line':
            p1, p2 = map(np.asarray, prm)
            t, s   = line(p1, p2, v, Ts)
        else:                           # arc
            center, r, a0, a1 = prm
            t, s = arc(np.asarray(center), r, a0, a1, v, Ts)
        trajs.append(t);  vels.append(s)
    traj = np.vstack(trajs)
    vref = np.concatenate(vels)
    return traj, vref

# ---------------------- 3. 建立兩軸離散模型 ------------
def build_axis(ax):
    a = AXES[ax]
    A  = np.array([[0, Kb], [0, -a['B']/a['J']]])
    Bv = np.array([[0], [a['Kt']/a['J']]])
    C  = np.array([[1, 0]])
    D  = np.zeros((1,1))
    ss_c = ss(A, Bv, C, D)
    ss_d = c2d(ss_c, Ts)
    return ss_d.A, ss_d.B.flatten()

AX_MAT = {ax: build_axis(ax) for ax in AXES}

# ---------------------- 4. 兩軸閉環模擬 -----------------
def simulate(traj, vref):
    """傳回 pos_log(N,2) 及 e(N)"""
    N   = len(traj)
    x   = {ax: np.zeros(2) for ax in AXES}     # [pos, ω] each
    iv  = {ax: 0.0 for ax in AXES}             # ∫e_v
    pos = np.zeros((N,2))

    for k in range(N):
        for i, ax in enumerate(('X','Y')):
            A,B = AX_MAT[ax]
            s   = AXES[ax]
            # 量測
            pos_mm = x[ax][0];   vel_mm = Kb*x[ax][1]
            # 外層 P
            e_p  = traj[k,i] - pos_mm
            u_p  = s['Kpp'] * e_p
            # 內層 PI
            v_cmd = vref[k] + u_p
            e_v   = v_cmd - vel_mm
            iv[ax]+= e_v*Ts
            u_v   = s['Kvp']*e_v + s['Kvi']*iv[ax]
            # 前饋
            u_ff  = s['Kr'] * vref[k]
            u_tot = u_ff + u_v
            # 狀態更新
            x[ax] = A@x[ax] + B*u_tot
            pos[k,i] = x[ax][0]

    err = np.linalg.norm(traj - pos, axis=1)   # 輪廓誤差
    return pos, err

# ---------------------- 5. 迴圈測 3 個 Vf ---------------
results = {}
for vf in Vf_list:
    ref, vr = build_path(vf)
    act, er = simulate(ref, vr)
    results[vf] = dict(ref=ref, act=act, er=er,
        rmse=np.sqrt(np.mean(er**2)), mxe=np.max(abs(er)))
    print(f"V={vf:>3} mm/min  RMSE={results[vf]['rmse']*1e3:6.2f} µm   "
          f"Max={results[vf]['mxe']*1e3:6.2f} µm")

# ---------------------- 6. 視覺化 -----------------------
plt.figure(figsize=(5,5))
for vf,r in results.items():
    plt.plot(r['ref'][:,0], r['ref'][:,1],'--', label=f'ref {vf}')
    plt.plot(r['act'][:,0], r['act'][:,1],  label=f'act {vf}')
plt.gca().set_aspect('equal', adjustable='box')
plt.title('Trajectory - reference vs actual');  plt.legend();  plt.xlabel('X (mm)');  plt.ylabel('Y (mm)')

plt.figure()
for vf,r in results.items():
    plt.plot(r['er']*1e3, label=f'{vf} mm/min')
plt.title('Contour-error history');  plt.xlabel('Sample #');  plt.ylabel('Error (µm)');  plt.legend()
plt.show()
