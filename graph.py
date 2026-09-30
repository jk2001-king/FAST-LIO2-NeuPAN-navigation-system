import numpy as np
import matplotlib.pyplot as plt


def main():
    # Table values used in the paper/presentation
    metrics = [
        "Success Rate [%]",
        "Final Pos. Error [m]",
        "Max Pos. Jump [m]",
        "Collision Count",
    ]

    fast_lio2 = np.array([50.0, 1.032, 5.399, 5.0], dtype=float)
    auto_toggle = np.array([90.0, 0.589, 2.389, 2.0], dtype=float)

    # Positive means "better" for presentation.
    # Success rate: increase is good.
    # Error / jump / collisions: reduction is good.
    improvement = np.array([
        (auto_toggle[0] - fast_lio2[0]) / fast_lio2[0] * 100.0,
        (fast_lio2[1] - auto_toggle[1]) / fast_lio2[1] * 100.0,
        (fast_lio2[2] - auto_toggle[2]) / fast_lio2[2] * 100.0,
        (fast_lio2[3] - auto_toggle[3]) / fast_lio2[3] * 100.0,
    ])

    plt.rcParams.update({
        "font.size": 14,
        "font.family": "DejaVu Sans",
    })

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))

    # --------------------------------------------------
    # Left: absolute metric comparison
    # --------------------------------------------------
    ax = axes[0]
    x = np.arange(len(metrics))
    width = 0.34

    ax.bar(x - width / 2, fast_lio2, width, label="FAST-LIO2 + Nav2", color="#d9534f")
    ax.bar(x + width / 2, auto_toggle, width, label="Auto Toggle", color="#2ca25f")

    ax.set_xticks(x)
    ax.set_xticklabels(metrics, fontsize=11, rotation=12, ha="right")
    ax.set_ylabel("Metric Value")
    ax.set_title("Absolute Performance Comparison", fontweight="bold", pad=12)
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    ax.legend(loc="upper right")
    ax.tick_params(axis="x", pad=10)

    # Put labels above bars
    for xpos, value in zip(x - width / 2, fast_lio2):
        ax.text(xpos, value + max(fast_lio2.max(), auto_toggle.max()) * 0.015,
                f"{value:.3g}", ha="center", va="bottom", fontsize=11)
    for xpos, value in zip(x + width / 2, auto_toggle):
        ax.text(xpos, value + max(fast_lio2.max(), auto_toggle.max()) * 0.015,
                f"{value:.3g}", ha="center", va="bottom", fontsize=11)

    # --------------------------------------------------
    # Right: improvement rate
    # --------------------------------------------------
    ax2 = axes[1]
    colors = ["#1f77b4", "#1f77b4", "#1f77b4", "#1f77b4"]
    bars = ax2.bar(x, improvement, color=colors, width=0.55)

    ax2.set_xticks(x)
    ax2.set_xticklabels(metrics, fontsize=11, rotation=12, ha="right")
    ax2.set_ylabel("Improvement (%)")
    ax2.set_title("Performance Improvement with Auto Toggle", fontweight="bold", pad=12)
    ax2.grid(axis="y", linestyle=":", alpha=0.5)
    ax2.axhline(0.0, color="black", linewidth=1.0)
    ax2.tick_params(axis="x", pad=10)

    for bar, value in zip(bars, improvement):
        ax2.text(
            bar.get_x() + bar.get_width() / 2.0,
            value + 1.0,
            f"{value:.1f}%",
            ha="center",
            va="bottom",
            fontsize=11,
            fontweight="bold",
        )

    fig.suptitle("Navigation Performance Comparison", fontsize=18, fontweight="bold", y=1.02)
    fig.subplots_adjust(bottom=0.28, wspace=0.28)

    output_path = "performance_comparison.png"
    plt.savefig(output_path, dpi=300, bbox_inches="tight")
    print(f"Saved figure to: {output_path}")
    plt.show()


if __name__ == "__main__":
    main()
