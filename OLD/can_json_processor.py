import sys
import json
import re
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout,QHBoxLayout, QTableWidget, QTableWidgetItem,
    QLabel, QTextEdit, QMessageBox, QLineEdit, QDialog
)
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtGui import QColor
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QFileDialog
from visualizer import CanNetworkVisualizer
import os

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5 import NavigationToolbar2QT as NavigationToolbar
class VcdViewer(QDialog):
    def __init__(self, vcd_path):
        super().__init__()
        self.setWindowTitle(f"VCD Viewer - {os.path.basename(vcd_path)}")
        self.resize(1000, 600)
        layout = QVBoxLayout(self)
        # Add matplotlib navigation toolbar for pan/zoom
        self.canvas = FigureCanvas(Figure(figsize=(10, 5)))
        self.toolbar = NavigationToolbar(self.canvas, self)
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        self.ax = self.canvas.figure.add_subplot(111)
        self._parse_and_plot(vcd_path)
    def _parse_and_plot(self, path):
        signals = {}         # code -> name
        signal_data = {}     # name -> list of (time, value)
        current_time = 0
        with open(path, 'r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                if line.startswith('$var'):
                    parts = line.split()
                    if len(parts) >= 5:
                        code = parts[3]
                        name = parts[4]
                        signals[code] = name
                        signal_data[name] = []
                elif line.startswith('#'):
                    try:
                        current_time = int(line[1:])
                    except ValueError:
                        pass
                elif line[0] in ('0', '1'):
                    value = int(line[0])
                    code = line[1:]
                    name = signals.get(code, code)
                    signal_data[name].append((current_time, value))
        max_time = 0
        # Plot square wave signals
        for i, (name, changes) in enumerate(reversed(list(signal_data.items()))):
            if not changes:
                continue
            times = []
            values = []
            last_time = 0
            last_val = 0
            for t, v in changes:
                times.extend([last_time, t])
                values.extend([last_val, last_val])
                last_time = t
                last_val = v
            times.append(last_time)
            values.append(last_val)
            max_time = max(max_time, last_time)
            self.ax.plot(times, [v * 0.1 + i * 0.15 for v in values], drawstyle='steps-post', label=name)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Signals')
        self.ax.set_title('VCD Signal Viewer')
        self.ax.legend(loc='upper right', bbox_to_anchor=(1.15, 1))
        self.ax.grid(True)
        self.ax.set_ylim(-1, len(signal_data) * 2)
        self.ax.set_xlim(0, max_time + 10)
        self.canvas.draw()



class CanJsonProcessor(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.input_packets = [] 
        self.all_packets = []

    def initUI(self):
        self.setWindowTitle('GUI FOR CARLA')
        self.setGeometry(100, 100, 900, 400)  

        # self.label = QLabel('Introdu JSON-ul CAN:', self)
        # self.label.setFont(QFont('Arial', 12))

        self.json_input = QTextEdit(self)
        self.json_input.setFont(QFont('Arial', 10))
        self.json_input.setPlaceholderText("Introdu JSON-ul aici... (poate fi si neformatat)")
        self.json_input.setStyleSheet("background-color: #f0f0f0; border: 1px solid #ccc; padding: 5px;")
        self.json_input.setAcceptRichText(False)
        self.json_input.setVisible(False)
        # ----- Butoane -------

        # Importă JSON
        self.import_button = QPushButton('📂', self)
        self.import_button.setToolTip("Importă fișier JSON")
        self.import_button.setFont(QFont('Arial', 12))
        self.import_button.clicked.connect(self.import_json)


        # # Exportă JSON
        # self.export_button = QPushButton('💾', self)
        # self.export_button.setToolTip("Exportă JSON-ul curent")
        # self.export_button.setFont(QFont('Arial', 12))
        # self.export_button.clicked.connect(self.export_json)

        # Tabel report/command
        self.toggle_table_button = QPushButton('📊', self)
        self.toggle_table_button.setToolTip("Comută tabelul report/command")
        self.toggle_table_button.setFont(QFont('Arial', 12))
        self.toggle_table_button.clicked.connect(self.togglerepo_comm)

        # Afișează tabelul de la Simulator
        self.show_table_button = QPushButton('🚘', self)
        self.show_table_button.setToolTip("Comută tabelul simulatorului")
        self.show_table_button.setFont(QFont('Arial', 12))
        self.show_table_button.clicked.connect(self.toggle_table)

        # Arată/Ascunde editor JSON
        self.toggle_input_button = QPushButton('📝', self)
        self.toggle_input_button.setToolTip("Arată/Ascunde editorul de JSON manual")
        self.toggle_input_button.setFont(QFont('Arial', 12))
        self.toggle_input_button.clicked.connect(self.toggle_json_input)

        self.network_button = QPushButton('🕸️', self)
        self.network_button.setToolTip("Deschide schema CAN")
        self.network_button.setFont(QFont('Arial', 12))
        self.network_button.clicked.connect(self.open_network_visualizer)

        # Proceseaza fisier VCD
        self.vcd_button = QPushButton('📈', self)
        self.vcd_button.setToolTip("Deschide și afișează fișier VCD")
        self.vcd_button.setFont(QFont('Arial', 12))
        self.vcd_button.clicked.connect(self.open_vcd_file)

        # Procesează JSON
        self.process_button = QPushButton('⚙️', self)
        self.process_button.setToolTip("Procesează conținutul JSON")
        self.process_button.setFont(QFont('Arial', 12))
        self.process_button.clicked.connect(self.process_json)
        self.process_button.setVisible(False)

        # ----------- Tabel 

        self.search_box = QLineEdit(self)
        self.search_box.setPlaceholderText("Caută în tabel...")
        self.search_box.textChanged.connect(self.filter_table)


        self.label_input = QLabel('📥 Pachete CAN Primite (INPUT)', self)

        self.table = QTableWidget()
        self.table.setColumnCount(10) 
        self.table.setHorizontalHeaderLabels([
            "ID", "Date", "Sursa", "Destinatie", "Nume", "Nivel", "Tip", "Perioada", "Data Size", "Carla Var"
        ])
        self.table.setVisible(True) 

        self.table.setColumnWidth(0, 60)   # ID
        self.table.setColumnWidth(9, 60)  # Date
        self.table.setColumnWidth(1, 120)  # Sursa
        self.table.setColumnWidth(2, 120)  # Destinatie
        self.table.setColumnWidth(3, 160)  # Nume
        self.table.setColumnWidth(4, 80)   # Nivel
        self.table.setColumnWidth(5, 80)   # Tip
        self.table.setColumnWidth(6, 80)   # Perioada
        self.table.setColumnWidth(7, 80)   # Data Size
        self.table.setColumnWidth(8, 100)  # Carla Var

        self.label_simulator = QLabel('🚗 Pachete CAN Procesate (SIMULATOR)', self)

        self.sim_table_command = QTableWidget()
        self.sim_table_command.setColumnCount(10)  
        self.sim_table_command.setHorizontalHeaderLabels([
            "ID", "Date", "Sursa", "Destinatie", "Nume", "Nivel", "Tip", "Perioada", "Data Size", "Carla Var"
        ])
        self.sim_table_command.setVisible(True)
        self.sim_table_command.setColumnWidth(0, 60)   # ID
        self.sim_table_command.setColumnWidth(9, 60)  # Date
        self.sim_table_command.setColumnWidth(1, 120)  # Sursa
        self.sim_table_command.setColumnWidth(2, 120)  # Destinatie
        self.sim_table_command.setColumnWidth(3, 160)  # Nume
        self.sim_table_command.setColumnWidth(4, 80)   # Nivel
        self.sim_table_command.setColumnWidth(5, 80)   # Tip
        self.sim_table_command.setColumnWidth(6, 80)   # Perioada
        self.sim_table_command.setColumnWidth(7, 80)   # Data Size
        self.sim_table_command.setColumnWidth(8, 100)  # Carla Var


        self.sim_table_report = QTableWidget()
        self.sim_table_report.setColumnCount(10)  
        self.sim_table_report.setHorizontalHeaderLabels([
            "ID", "Date", "Sursa", "Destinatie", "Nume", "Nivel", "Tip", "Perioada", "Data Size", "Carla Var"
        ])
        self.sim_table_report.setVisible(False)
        self.sim_table_report.setColumnWidth(0, 60)   # ID
        self.sim_table_report.setColumnWidth(9, 60)  # Date
        self.sim_table_report.setColumnWidth(1, 120)  # Sursa
        self.sim_table_report.setColumnWidth(2, 120)  # Destinatie
        self.sim_table_report.setColumnWidth(3, 160)  # Nume
        self.sim_table_report.setColumnWidth(4, 80)   # Nivel
        self.sim_table_report.setColumnWidth(5, 80)   # Tip
        self.sim_table_report.setColumnWidth(6, 80)   # Perioada
        self.sim_table_report.setColumnWidth(7, 80)   # Data Size
        self.sim_table_report.setColumnWidth(8, 100)  # Carla Var




        # Layout principal
        layout = QVBoxLayout()

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.import_button)
        # button_layout.addWidget(self.export_button)
        button_layout.addWidget(self.toggle_table_button)
        button_layout.addWidget(self.show_table_button)
        button_layout.addWidget(self.toggle_input_button)
        button_layout.addWidget(self.network_button)
        button_layout.addWidget(self.vcd_button)
        button_layout.addWidget(self.process_button)
        button_layout.addStretch()
        # layout.addWidget(self.table)

	
        table_layout = QHBoxLayout()
        table_layout.addWidget(self.label_input)
        table_layout.addWidget(self.label_simulator)
        

        sim_table_input_layout = QVBoxLayout()
        
        sim_table_input_layout.addWidget(self.table)
        sim_table_input_layout.addWidget(self.show_table_button)

        sim_table_simulator_layout = QVBoxLayout()
        sim_table_simulator_layout.addWidget(self.search_box) 
        sim_table_simulator_layout.addWidget(self.sim_table_report)
        sim_table_simulator_layout.addWidget(self.sim_table_command)

        # Layout principal pentru vizualizarea tabelelor
        table_view_layout = QHBoxLayout()
        table_view_layout.addWidget(self.table)  # Primul tabel (INPUT)
        table_view_layout.addLayout(sim_table_simulator_layout)  # Al doilea tabel + search box

        # Adăugăm la layout-ul principal
        layout.addLayout(button_layout)
        # layout.addWidget(QLabel(" JSON Input"))
        layout.addWidget(self.json_input)
        layout.addLayout(table_layout)
        layout.addLayout(table_view_layout)

        self.setLayout(layout)

	
    def open_vcd_file(self):
        path, _ = QFileDialog.getOpenFileName(self, "Deschide fișier VCD", "", "VCD Files (*.vcd)")
        if not path:
            return
        viewer = VcdViewer(path)
        viewer.exec_()

    def toggle_json_input(self):
        self.json_input.setVisible(not self.json_input.isVisible())
        self.process_button.setVisible(not self.process_button.isVisible())


    def import_json(self):
        file_name, _ = QFileDialog.getOpenFileName(self, "Deschide fișier JSON", "", "JSON Files (*.json)")
        if file_name:
            with open(file_name, "r") as f:
                json_content = f.read()
                self.json_input.setText(json_content)
            self.process_json()


    def add_packet_to_table_report(self, packet):
        can_id = str(packet["can_id"])  # Convertim ID-ul la string pentru consistență

        # Căutăm dacă ID-ul există deja în tabel
        row_to_update = -1
        for row in range(self.sim_table_report.rowCount()):
            existing_id = self.sim_table_report.item(row, 0)
            if existing_id and existing_id.text() == can_id:
                row_to_update = row
                break  # Găsit -> oprim căutarea

        if row_to_update == -1:
            row_to_update = self.sim_table_report.rowCount()
            self.sim_table_report.insertRow(row_to_update)

        # Suprascriem datele din rândul găsit/creat
        self.sim_table_report.setItem(row_to_update, 0, QTableWidgetItem(can_id))
        self.sim_table_report.setItem(row_to_update, 1, QTableWidgetItem(str(packet.get("data", "N/A"))))  
        self.sim_table_report.setItem(row_to_update, 2, QTableWidgetItem(packet.get("src", "N/A")))
        self.sim_table_report.setItem(row_to_update, 3, QTableWidgetItem(packet.get("dst", "N/A")))
        self.sim_table_report.setItem(row_to_update, 4, QTableWidgetItem(packet.get("name", "N/A")))
        self.sim_table_report.setItem(row_to_update, 5, QTableWidgetItem(packet.get("level", "N/A")))
        self.sim_table_report.setItem(row_to_update, 6, QTableWidgetItem(packet.get("type", "N/A")))
        self.sim_table_report.setItem(row_to_update, 7, QTableWidgetItem(str(packet.get("period", "N/A"))))
        self.sim_table_report.setItem(row_to_update, 8, QTableWidgetItem(str(packet.get("datasize", "N/A"))))
        self.sim_table_report.setItem(row_to_update, 9, QTableWidgetItem(packet.get("carlaVar", "N/A")))

        # print(f"Pachet CAN procesat: {packet}")

        # print(f"Pachet CAN afisat in raport: {packet}")

    def get_row_color_by_can_id(self, can_id):
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
        return color_map.get(str(can_id), None)  # None = nu colorăm


    def add_packet_to_table(self, packet):
        # table = self.sim_table_command
        # row_position = self.sim_table_command.rowCount()
        if self.all_packets and self.all_packets[-1] == packet:
            return
        self.all_packets.append(packet)  # Stochează pachetele pentru filtrare
        # self.update_table(self.all_packets)

        self.sim_table_command.insertRow(0)
        self.sim_table_command.setItem(0, 0, QTableWidgetItem(str(packet["can_id"])))
        self.sim_table_command.setItem(0, 1, QTableWidgetItem(str(packet.get("data", "N/A"))))  
        self.sim_table_command.setItem(0, 2, QTableWidgetItem(packet.get("src", "N/A")))
        self.sim_table_command.setItem(0, 3, QTableWidgetItem(packet.get("dst", "N/A")))
        self.sim_table_command.setItem(0, 4, QTableWidgetItem(packet.get("name", "N/A")))
        self.sim_table_command.setItem(0, 5, QTableWidgetItem(packet.get("level", "N/A")))
        self.sim_table_command.setItem(0, 6, QTableWidgetItem(packet.get("type", "N/A")))
        self.sim_table_command.setItem(0, 7, QTableWidgetItem(str(packet.get("period", "N/A"))))
        self.sim_table_command.setItem(0, 8, QTableWidgetItem(str(packet.get("datasize", "N/A"))))
        self.sim_table_command.setItem(0, 9, QTableWidgetItem(packet.get("carlaVar", "N/A")))
        if packet["can_id"] == "440":
            for col in range(self.sim_table_command.columnCount()):
                self.sim_table_command.item(0, col).setBackground(Qt.red)
        # print(f"Pachet CAN afisat in tabel: {packet}")
        row_color = self.get_row_color_by_can_id(packet["can_id"])
        if row_color:
            for col in range(self.sim_table_command.columnCount()):
                item = self.sim_table_command.item(0, col)
                if item:
                    item.setBackground(QColor(row_color))

        if hasattr(self, 'network_window') and self.network_window.isVisible():
            print(f"Packet {packet['can_id']} from {packet['src']} to {packet['dst']}")

            src = packet.get("src")
            dst = packet.get("dst")
            can_id = packet.get("can_id")

            # Trece întâi prin CGW cu culoare în funcție de ID
            if src in self.network_window.modules and dst in self.network_window.modules:
                self.network_window.send_packet(src, "CGW", can_id)
                QTimer.singleShot(800, lambda: self.network_window.send_packet("CGW", dst, can_id))




    def update_table(self, packets):
        self.sim_table_command.setRowCount(0)

        for packet in packets:
            row_position = self.table.rowCount()
            self.sim_table_command.insertRow(row_position)

            self.sim_table_command.setItem(row_position, 0, QTableWidgetItem(str(packet["can_id"])))
            self.sim_table_command.setItem(row_position, 1, QTableWidgetItem(str(packet["data"])))
            self.sim_table_command.setItem(row_position, 2, QTableWidgetItem(packet["src"]))
            self.sim_table_command.setItem(row_position, 3, QTableWidgetItem(packet["dst"]))
            self.sim_table_command.setItem(row_position, 4, QTableWidgetItem(packet["name"]))
            self.sim_table_command.setItem(row_position, 5, QTableWidgetItem(packet["level"]))
            self.sim_table_command.setItem(row_position, 6, QTableWidgetItem(packet["type"]))
            self.sim_table_command.setItem(row_position, 7, QTableWidgetItem(str(packet.get("period", "N/A"))))
            self.sim_table_command.setItem(row_position, 8, QTableWidgetItem(str(packet.get("datasize", "N/A"))))
            self.sim_table_command.setItem(row_position, 9, QTableWidgetItem(packet["carlaVar"]))
            row_color = self.get_row_color_by_can_id(packet["can_id"])
            if row_color:
                for col in range(self.sim_table_command.columnCount()):
                    item = self.sim_table_command.item(row_position, col)
                    if item:
                        item.setBackground(QColor(row_color))  


    def filter_table(self):
        search_text = self.search_box.text().lower()
        filtered_packets = [
            p for p in self.all_packets if 
            search_text in str(p["can_id"]).lower() or
            search_text in str(p["data"]).lower() or
            search_text in str(p["src"]).lower() or
            search_text in str(p["dst"]).lower() or
            search_text in str(p["name"]).lower() or
            search_text in str(p["level"]).lower() or
            search_text in str(p["type"]).lower() or
            search_text in str(p["period"]).lower() or
            search_text in str(p["datasize"]).lower() or
            search_text in str(p["carlaVar"]).lower()
        ]
        self.update_table(filtered_packets)

    def toggle_table(self):
        self.table.setVisible(not self.table.isVisible()) 
        self.label_input.setVisible(not self.label_input.isVisible())

    def togglerepo_comm(self):
        self.sim_table_command.setVisible(not self.sim_table_command.isVisible())
        self.search_box.setVisible(not self.search_box.isVisible())
        self.sim_table_report.setVisible(not self.sim_table_report.isVisible())

    def process_json(self):
        json_text = self.json_input.toPlainText()

        try:
            json_data = json.loads(json_text)
        except json.JSONDecodeError:
            json_text = self.fix_json(json_text)
            try:
                json_data = json.loads(json_text)
            except json.JSONDecodeError:
                QMessageBox.critical(self, "Eroare", QMessageBox.Ok)
                return


        for can_id, details in json_data.items():
            self.add_command_packet(can_id, details)  
            # row_position = self.table.rowCount()
            self.table.insertRow(0)

            self.table.setItem(0, 0, QTableWidgetItem(str(can_id)))  # ID
            self.table.setItem(0, 1, QTableWidgetItem(str(details.get("data", "N/A")))) # Date
            self.table.setItem(0, 2, QTableWidgetItem(details.get("source", "N/A")))  # Sursa
            self.table.setItem(0, 3, QTableWidgetItem(details.get("execution", "N/A")))  # Destinatie
            self.table.setItem(0, 4, QTableWidgetItem(details.get("name", "N/A")))  # Nume
            self.table.setItem(0, 5, QTableWidgetItem(details.get("level", "N/A")))  # Nivel
            self.table.setItem(0, 6, QTableWidgetItem(details.get("type", "N/A")))  # Tip
            self.table.setItem(0, 7, QTableWidgetItem(str(details.get("period", "N/A"))))  # Perioada
            self.table.setItem(0, 8, QTableWidgetItem(str(details.get("datasize", "N/A"))))  # Data Size
            self.table.setItem(0, 9, QTableWidgetItem(details.get("carlaVar", "N/A")))  # Carla Var

            if hasattr(self, 'network_window') and self.network_window.isVisible():
                if details.get("level") == "command" and details.get("carlaVar") is not None:
                    src = "Tester"
                    dst = details.get("execution")
                    can_id = can_id
                    self.network_window.send_packet(src, "CGW", can_id)
                    QTimer.singleShot(800, lambda dst=dst, can_id=can_id: self.network_window.send_packet("CGW", dst, can_id))

    def add_command_packet(self, can_id, details):
        if details["level"] == "command":
            data_value = details.get("data", None)
            packet = {
                "can_id": can_id,
                "src": details.get("source"),
                "dst": details.get("execution"),
                "name": details.get("name"),
                "level": details.get("level"),
                "type": details.get("type"),
                "period": details.get("period"),
                "datasize": details.get("datasize"),
                "min": details.get("min"),
                "max": details.get("max"),
                "carlaVar": details.get("carlaVar"),
                "data": data_value  
            }

            self.input_packets.append(packet)

    def fix_json(self, text):

        if text.endswith("},}"):
            text = text[:-2]
        if text.endswith("},"):
            text = text[:-1]    

        if not text.startswith("{"):
            text = "{" + text
        if not text.endswith("},}"):
            text = text + "}"

        print(text)
        return text
  
    def open_network_visualizer(self):
        if not hasattr(self, 'network_window') or self.network_window is None:
            self.network_window = CanNetworkVisualizer()
        self.network_window.show()


