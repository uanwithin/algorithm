"""线性系统模态练习：按 README 顺序完成 TODO，预测填写在 REPORT.md。"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

Array = NDArray[np.float64]

A = np.array([[-2.0, 1.0], [1.0, -2.0]])
INITIAL_STATES = {
    "same": np.array([1.0, 1.0]),
    "opposite": np.array([1.0, -1.0]),
    "mixed": np.array([1.0, 0.0]),
}
T_END = 5.0
DEFAULT_DT = 0.01
ERROR_STEPS = (0.1, 0.05, 0.01)
FIGURES_DIR = Path(__file__).resolve().parent / "figures"


def make_time_grid(dt: float, t_end: float) -> Array:
    """包含起点和终点；本练习要求 dt 能整除 t_end。"""
    if not np.isfinite(dt) or not np.isfinite(t_end) or dt <= 0 or t_end < 0:
        raise ValueError("dt 必须为有限正数，t_end 必须为有限非负数")
    steps = round(t_end / dt)
    if not np.isclose(steps * dt, t_end, rtol=1e-10, atol=1e-12):
        raise ValueError("本练习要求 dt 能整除 t_end")
    return np.linspace(0.0, t_end, steps + 1)


def simulate_euler(A: Array, x0: Array, dt: float, t_end: float) -> tuple[Array, Array]:
    """返回 times: (N+1,)，states: (N+1, 2)，每行对应一个时刻。"""
    times = make_time_grid(dt, t_end)
    states = np.zeros((len(times), len(x0)), dtype=float)
    states[0] = x0.copy()
    # 这里应该是 [0, t_end]
    for idx in range(1, len(times)):
        last_state = states[idx - 1]
        dot_last_state = A @ last_state
        states[idx] = last_state + dot_last_state * dt

    return times, states


# 按列排列特征向量的矩阵 T，以及与列顺序对应的特征值 eigenvalues。
def build_modal_basis() -> tuple[Array, Array]:
    """返回 T: (2, 2)，eigenvalues: (2,)，列顺序与特征值顺序对应。"""
    v1 = np.array([1.0, 1.0])
    v2 = np.array([1.0, -1.0])
    T = np.column_stack((v1, v2))
    eigenvalues = np.array([-1.0, -3.0])

    return T, eigenvalues


def check_modal_basis(A: Array, T: Array, eigenvalues: Array) -> None:
    """检查基可逆以及 AT=TD，并打印残差。"""
    if T.shape != A.shape or eigenvalues.shape != (A.shape[0],):
        raise ValueError("T 或 eigenvalues 的形状不正确")
    if np.linalg.matrix_rank(T) != A.shape[0]:
        raise ValueError("特征向量矩阵必须可逆")
    D = np.diag(eigenvalues)
    residual = np.linalg.norm(A @ T - T @ D, ord="fro")
    print(f"||AT - TD||_F = {residual:.6e}")
    if not np.allclose(A @ T, T @ D, rtol=1e-10, atol=1e-12):
        raise ValueError("AT=TD 检查失败，请检查特征值与特征向量的对应关系")


# 模态坐标轨迹 modes    和    重建后的原状态轨迹 states
#  特征向量空间下的坐标           原始坐标系下的坐标
def solve_modal(
    T: Array, eigenvalues: Array, x0: Array, times: Array
) -> tuple[Array, Array]:
    """返回 modes 和 states，形状均为 (len(times), 2)。"""
    # T_inv = np.linalg.inv(T)

    # z0 = T_inv @ x0
    z0 = np.linalg.solve(T, x0)
    modes = np.zeros((len(times), 2))
    states = np.zeros((len(times), 2))

    for idx, time in enumerate(times):
        modes[idx] = np.array(
            [
                np.exp(eigenvalues[0] * time) * z0[0],
                np.exp(eigenvalues[1] * time) * z0[1],
            ]
        )
        states[idx] = T @ modes[idx]

    return modes, states


# 比较两种方法得到的状态轨迹，返回它们在所有采样时刻中最大的偏差。
def max_state_error(actual: Array, reference: Array) -> float:
    """同一时间网格上，先对状态差求二范数，再对时间取最大值。"""
    if actual.shape != reference.shape or actual.ndim != 2:
        raise ValueError("两组轨迹必须具有相同的二维形状")
    difference = actual - reference
    norms = np.linalg.norm(difference, ord=2, axis=1)
    return np.max(norms)
    # TODO 4：计算 README 中定义的误差，不要只比较终点。
    raise NotImplementedError("TODO 4：完成 max_state_error")


def solve_matrix_exponential(A: Array, x0: Array, times: Array) -> Array:
    """选做核对：返回 (len(times), 2) 的状态轨迹。"""
    # TODO 6：在函数内导入 scipy.linalg.expm，再逐时刻求解。
    raise NotImplementedError("TODO 6：完成矩阵指数核对")


def solve_nilpotent(A: Array, x0: Array, times: Array) -> Array:
    """选做推导：利用 A²=0 返回解析轨迹，不直接调用 expm。"""
    # TODO 7：从矩阵指数的幂级数推导，并填写解析轨迹。
    raise NotImplementedError("TODO 7：完成无法对角化系统的解析解")


def plot_trajectories(
    times: Array,
    trajectories: dict[str, Array],
    filename: str,
    *,
    coordinates: tuple[str, str] = ("x1", "x2"),
) -> None:
    """通用绘图：左侧分量时间曲线，右侧状态平面，可叠加多种方法。"""
    import matplotlib.pyplot as plt

    fig, (time_ax, phase_ax) = plt.subplots(1, 2, figsize=(11, 4))
    styles = ("-", "--", ":", "-.")
    for index, (label, states) in enumerate(trajectories.items()):
        style = styles[index % len(styles)]
        for component, color in enumerate(("tab:blue", "tab:orange")):
            time_ax.plot(
                times,
                states[:, component],
                color=color,
                linestyle=style,
                label=f"{label}: {coordinates[component]}",
            )
        phase_ax.plot(states[:, 0], states[:, 1], linestyle=style, label=label)
        phase_ax.scatter(states[0, 0], states[0, 1], marker="o", s=25)
    time_ax.set(xlabel="t", ylabel="Coordinate")
    phase_ax.set(xlabel=coordinates[0], ylabel=coordinates[1])
    phase_ax.set_aspect("equal", adjustable="datalim")
    for ax in (time_ax, phase_ax):
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.suptitle(filename)
    fig.tight_layout()
    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    path = FIGURES_DIR / f"{filename}.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print(f"图像已保存：{path}")


def run_euler() -> None:
    for name, x0 in INITIAL_STATES.items():
        times, states = simulate_euler(A, x0, DEFAULT_DT, T_END)
        plot_trajectories(times, {"Euler": states}, f"euler_{name}")


def run_modal() -> None:
    T, eigenvalues = build_modal_basis()
    check_modal_basis(A, T, eigenvalues)
    times = make_time_grid(DEFAULT_DT, T_END)
    for name, x0 in INITIAL_STATES.items():
        modes, states = solve_modal(T, eigenvalues, x0, times)
        print(f"{name}: z0 = {modes[0]}")
        plot_trajectories(
            times, {"Modal": modes}, f"modes_{name}", coordinates=("z1", "z2")
        )
        plot_trajectories(times, {"Reconstructed": states}, f"modal_state_{name}")


def run_comparison(*, include_expm: bool = False) -> None:
    T, eigenvalues = build_modal_basis()
    check_modal_basis(A, T, eigenvalues)
    print("case\tdt\tEuler vs modal max error")
    for name, x0 in INITIAL_STATES.items():
        for dt in ERROR_STEPS:
            times, euler_states = simulate_euler(A, x0, dt, T_END)
            _, modal_states = solve_modal(T, eigenvalues, x0, times)
            error = max_state_error(euler_states, modal_states)
            print(f"{name}\t{dt:g}\t{error:.6e}")
            trajectories = {"Euler": euler_states, "Modal": modal_states}
            if include_expm:
                expm_states = solve_matrix_exponential(A, x0, times)
                trajectories["expm"] = expm_states
                print(
                    f"  expm vs modal: {max_state_error(expm_states, modal_states):.6e}"
                )
            plot_trajectories(times, trajectories, f"compare_{name}_dt_{dt:g}")


# 特征向量的长度改变后，模态坐标会改变，但重建出的原状态轨迹不变
def run_scaling() -> None:
    T, eigenvalues = build_modal_basis()
    check_modal_basis(A, T, eigenvalues)

    scaled_T = T.copy()
    scaled_T[:, 0] *= 3.0
    check_modal_basis(A, scaled_T, eigenvalues)

    times = make_time_grid(DEFAULT_DT, T_END)

    for name, x0 in INITIAL_STATES.items():
        # 两次求解都会根据传入的基，重新计算模态初值。
        modes, states = solve_modal(T, eigenvalues, x0, times)
        scaled_modes, scaled_states = solve_modal(scaled_T, eigenvalues, x0, times)

        print(f"\n初始状态：{name}, x0 = {x0}")
        print(f"原模态初值：{modes[0]}")
        print(f"新模态初值：{scaled_modes[0]}")

        error = max_state_error(scaled_states, states)
        print(f"重建状态的最大误差：{error:.6e}")

        # 比较模态坐标：第一模态应缩小为原来的三分之一。
        plot_trajectories(
            times,
            {"Original basis": modes, "Scaled basis": scaled_modes},
            f"scaling_modes_{name}",
            coordinates=("z1", "z2"),
        )

        # 比较原状态：两条轨迹应重合。
        plot_trajectories(
            times,
            {"Original basis": states, "Scaled basis": scaled_states},
            f"scaling_states_{name}",
        )
    # TODO 5：先记录预测，再复制 T，将第一列放大三倍。
    # 对三个初值分别用新旧基调用 solve_modal，重新分解初值。
    # 打印新旧模态初值，绘制模态与重建状态的对比图，并计算状态误差。
    # 不要原地修改原来的 T，也不要沿用旧的模态初值。
    # raise NotImplementedError("TODO 5：完成特征向量长度实验")


def run_nilpotent() -> None:
    nilpotent_A = np.array([[0.0, 1.0], [0.0, 0.0]])
    x0 = np.array([0.0, 1.0])
    times, euler_states = simulate_euler(nilpotent_A, x0, DEFAULT_DT, T_END)
    exact_states = solve_nilpotent(nilpotent_A, x0, times)
    print(f"Euler vs exact: {max_state_error(euler_states, exact_states):.6e}")
    plot_trajectories(
        times, {"Euler": euler_states, "Exact": exact_states}, "nilpotent"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--part",
        choices=("euler", "modal", "compare", "scaling", "expm", "nilpotent"),
        default="euler",
        help="选择实验阶段，默认 euler",
    )
    args = parser.parse_args()
    runners = {
        "euler": run_euler,
        "modal": run_modal,
        "compare": run_comparison,
        "scaling": run_scaling,
        "expm": lambda: run_comparison(include_expm=True),
        "nilpotent": run_nilpotent,
    }
    print("请先在 REPORT.md 保留本阶段的运行前预测。")
    try:
        runners[args.part]()
    except NotImplementedError as exc:
        print(f"尚未完成：{exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
