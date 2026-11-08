"""Swift browser 3D model and a separate matplotlib XYZ slider window."""
class SwiftViewer:
    def __init__(self, model, controller):
        self.robot = model.robot
        self.controller = controller

    def run(self):
        try:
            import swift
            import matplotlib.pyplot as plt
            from matplotlib.widgets import Slider
        except ImportError as exc:
            raise RuntimeError("Install simulation extras and a GUI-capable matplotlib backend") from exc
        env = swift.Swift()
        env.launch(realtime=True)
        env.add(self.robot)
        current = self.controller.kin.fk(self.controller.q)[:3, 3]
        fig, ax = plt.subplots(figsize=(9, 3))
        plt.subplots_adjust(left=0.17, bottom=0.2, top=0.82)
        ax.axis("off")
        status = ax.text(0.01, 0.75, "Move XYZ sliders (metres)", transform=ax.transAxes)
        sliders = []
        for i, label in enumerate("XYZ"):
            sa = fig.add_axes([0.20, 0.59 - i*0.16, 0.68, 0.08])
            sliders.append(Slider(sa, label, -0.40, 0.40, valinit=float(current[i]), valstep=0.005))

        def update(_):
            result = self.controller.move_tcp([s.val for s in sliders])
            status.set_text(f"{result.reason}; position error={result.position_error_m} m")
            fig.canvas.draw_idle()
            if result.accepted:
                env.step(0.01)

        for slider in sliders:
            slider.on_changed(update)
        plt.show(block=True)
        env.close()
