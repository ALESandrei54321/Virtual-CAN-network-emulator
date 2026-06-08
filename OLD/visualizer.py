from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsRectItem, QGraphicsEllipseItem
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QBrush, QColor


class CanNetworkVisualizer(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.setWindowTitle("CAN Network Visualizer")
        self.setGeometry(100, 100, 500, 400)

        # Module CAN
        self.modules = {
            "CGW": self.add_module(200, 20, "CGW"),
            "Tester": self.add_module(400, 20, "Tester"),
            "body": self.add_module(100, 150, "body"),
            "chasis": self.add_module(250, 150, "chasis"),
            "power train": self.add_module(400, 150, "power train"),
        }

        # Gestionare pachete active
        self.active_packets = []
        self.packet_timer = QTimer()
        self.packet_timer.timeout.connect(self.animate_packets)
        self.packet_timer.start(30)

    def add_module(self, x, y, label):
        rect = QGraphicsRectItem(x, y, 80, 40)
        rect.setBrush(QBrush(QColor("#cce5ff")))
        self.scene.addItem(rect)

        # Adaugă textul deasupra dreptunghiului
        text = self.scene.addText(label)
        text.setDefaultTextColor(QColor("black"))
        text.setPos(x + 10, y - 20)

        return rect

    def get_color_for_id(self, can_id):
        color_map = {
        "26":  "#e74c3c",   # brake
        "47":  "#3498db",   # throttle
        "88":  "#1abc9c",   # steer
        "109": "#f39c12",   # gear
        "440": "#c0392b",   # ignition
        "457": "#b71540",   # parking brake
        "131": "#9b59b6",   # lightTurn
        "423": "#f1c40f",   # lightFront
        "433": "#e67e22",   # passing
        "1200": "#ff5e57",  # collision
        "2100": "#95a5a6",  # manual
        "1313": "#00cec9"   # lane departure sound
        }
        return color_map.get(str(can_id), "#6c757d")  # gri default


    def send_packet(self, src, dst, can_id=None):
        if src not in self.modules or dst not in self.modules:
            print(f"❌ Modul lipsă: {src} sau {dst}")
            return

        start = self.modules[src].sceneBoundingRect().center()
        end = self.modules[dst].sceneBoundingRect().center()

        color = self.get_color_for_id(can_id)

        packet = QGraphicsEllipseItem(0, 0, 12, 12)
        packet.setBrush(QBrush(QColor(color)))
        packet.setPos(start)
        self.scene.addItem(packet)

        steps = 25
        dx = (end.x() - start.x()) / steps
        dy = (end.y() - start.y()) / steps

        self.active_packets.append({
            "item": packet,
            "dx": dx,
            "dy": dy,
            "count": 0,
            "steps": steps
        })


    def animate_packets(self):
        for packet in self.active_packets[:]:  # Copie ca să putem elimina
            item = packet["item"]
            if packet["count"] < packet["steps"]:
                item.moveBy(packet["dx"], packet["dy"])
                packet["count"] += 1
            else:
                self.scene.removeItem(item)
                self.active_packets.remove(packet)