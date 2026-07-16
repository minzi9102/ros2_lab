import argparse
import math
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from .handwriting_path import (
    compile_strokes,
    load_handwriting,
    path_length,
    save_handwriting,
)


class MouseHandwritingGui:
    def __init__(
        self,
        root: tk.Tk,
        *,
        output_path: str,
        writing_width_m: float,
        writing_height_m: float,
    ) -> None:
        self.root = root
        self.output_path = output_path
        self.writing_width_m = writing_width_m
        self.writing_height_m = writing_height_m
        self.strokes: list[list[tuple[float, float]]] = []
        self._active_stroke: list[tuple[float, float]] | None = None
        self.status = tk.StringVar(value="拖动鼠标左键开始书写；此工具不会连接机器人。")

        root.title("UR3e 离线鼠标手写")
        toolbar = ttk.Frame(root, padding=8)
        toolbar.pack(fill=tk.X)
        for label, command in (
            ("打开", self.open_file),
            ("保存", self.save_file),
            ("撤销上一笔", self.undo),
            ("清空", self.clear),
            ("处理后预览", self.preview),
        ):
            ttk.Button(toolbar, text=label, command=command).pack(
                side=tk.LEFT, padx=3
            )
        ttk.Label(
            toolbar,
            text=(
                f"目标范围 {writing_width_m * 1000:.1f} × "
                f"{writing_height_m * 1000:.1f} mm"
            ),
        ).pack(side=tk.RIGHT)

        self.canvas = tk.Canvas(
            root,
            background="white",
            highlightthickness=1,
            highlightbackground="#777777",
            cursor="crosshair",
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.canvas.bind("<ButtonPress-1>", self._start_stroke)
        self.canvas.bind("<B1-Motion>", self._continue_stroke)
        self.canvas.bind("<ButtonRelease-1>", self._finish_stroke)
        self.canvas.bind("<Configure>", lambda _event: self._redraw())
        ttk.Label(root, textvariable=self.status, padding=(8, 0, 8, 8)).pack(
            fill=tk.X
        )

    def _normalized_point(self, x: float, y: float) -> tuple[float, float]:
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        return (
            max(0.0, min(1.0, x / width)),
            max(0.0, min(1.0, y / height)),
        )

    def _start_stroke(self, event) -> None:
        self._active_stroke = [self._normalized_point(event.x, event.y)]

    def _continue_stroke(self, event) -> None:
        if self._active_stroke is None:
            return
        point = self._normalized_point(event.x, event.y)
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        previous = self._active_stroke[-1]
        pixel_distance = math.hypot(
            (point[0] - previous[0]) * width,
            (point[1] - previous[1]) * height,
        )
        if pixel_distance >= 1.0:
            self._active_stroke.append(point)
            self._redraw()

    def _finish_stroke(self, event) -> None:
        if self._active_stroke is None:
            return
        self._continue_stroke(event)
        if len(self._active_stroke) >= 2:
            self.strokes.append(self._active_stroke)
            self.status.set(f"已记录 {len(self.strokes)} 笔。")
        else:
            self.status.set("忽略没有移动的单击。")
        self._active_stroke = None
        self._redraw()

    def _redraw(self) -> None:
        self.canvas.delete("stroke")
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        strokes = self.strokes + ([self._active_stroke] if self._active_stroke else [])
        for stroke in strokes:
            coordinates = [
                coordinate
                for x, y in stroke
                for coordinate in (x * width, y * height)
            ]
            if len(coordinates) >= 4:
                self.canvas.create_line(
                    *coordinates,
                    fill="#111111",
                    width=3,
                    capstyle=tk.ROUND,
                    joinstyle=tk.ROUND,
                    tags="stroke",
                )

    def open_file(self) -> None:
        path = filedialog.askopenfilename(
            title="打开笔迹",
            initialfile=Path(self.output_path).name,
            filetypes=(("Handwriting JSON", "*.json"), ("All files", "*")),
        )
        if not path:
            return
        try:
            self.strokes = load_handwriting(path)
        except (OSError, ValueError) as exc:
            messagebox.showerror("无法打开", str(exc), parent=self.root)
            return
        self.output_path = path
        self._redraw()
        self.status.set(f"已打开 {path}，共 {len(self.strokes)} 笔。")

    def save_file(self) -> None:
        path = filedialog.asksaveasfilename(
            title="保存笔迹",
            initialfile=Path(self.output_path).name,
            defaultextension=".json",
            filetypes=(("Handwriting JSON", "*.json"),),
        )
        if not path:
            return
        try:
            save_handwriting(path, self.strokes)
        except (OSError, ValueError) as exc:
            messagebox.showerror("无法保存", str(exc), parent=self.root)
            return
        self.output_path = path
        self.status.set(f"已保存 {path}。")

    def undo(self) -> None:
        if self.strokes:
            self.strokes.pop()
            self._redraw()
        self.status.set(f"当前 {len(self.strokes)} 笔。")

    def clear(self) -> None:
        self.strokes.clear()
        self._active_stroke = None
        self._redraw()
        self.status.set("画布已清空。")

    def preview(self) -> None:
        try:
            compiled = compile_strokes(
                self.strokes,
                writing_width_m=self.writing_width_m,
                writing_height_m=self.writing_height_m,
            )
        except ValueError as exc:
            messagebox.showerror("无法预览", str(exc), parent=self.root)
            return
        window = tk.Toplevel(self.root)
        window.title("处理后轨迹预览（非机器人执行）")
        canvas = tk.Canvas(window, width=700, height=500, background="white")
        canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        all_points = [point for stroke in compiled for point in stroke]
        limit_x = max(abs(point[0]) for point in all_points) or 1.0
        limit_y = max(abs(point[1]) for point in all_points) or 1.0
        scale = min(320.0 / limit_x, 220.0 / limit_y)
        for index, stroke in enumerate(compiled):
            coordinates = [
                coordinate
                for x, y in stroke
                for coordinate in (350.0 + x * scale, 250.0 - y * scale)
            ]
            canvas.create_line(
                *coordinates,
                fill="#1f5fbf",
                width=3,
                capstyle=tk.ROUND,
                joinstyle=tk.ROUND,
            )
            start_x, start_y = coordinates[:2]
            canvas.create_text(start_x, start_y, text=str(index + 1), anchor=tk.SE)
        ttk.Label(
            window,
            text=(
                f"{len(compiled)} 笔，{sum(len(stroke) for stroke in compiled)} 点，"
                f"总长度 {path_length(compiled) * 1000:.1f} mm"
            ),
            padding=8,
        ).pack()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description="Offline mouse handwriting editor")
    parser.add_argument("--output", default="handwriting.json")
    parser.add_argument("--writing-width-mm", type=float, default=10.0)
    parser.add_argument("--writing-height-mm", type=float, default=10.0)
    parsed = parser.parse_args(args)
    if parsed.writing_width_mm <= 0.0 or parsed.writing_height_mm <= 0.0:
        parser.error("writing dimensions must be positive")
    root = tk.Tk()
    MouseHandwritingGui(
        root,
        output_path=parsed.output,
        writing_width_m=parsed.writing_width_mm / 1000.0,
        writing_height_m=parsed.writing_height_mm / 1000.0,
    )
    root.minsize(640, 480)
    root.geometry("900x650")
    root.mainloop()


if __name__ == "__main__":
    main()
