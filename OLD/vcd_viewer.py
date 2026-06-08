import os

from PyQt5.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QTableWidget, QTableWidgetItem
)
from PyQt5.QtGui import QColor
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas,
    NavigationToolbar2QT as NavigationToolbar
)
from matplotlib.figure import Figure
import matplotlib.patches as patches
import json
from constants import CAN_FIELDS, FIELD_COLORS


FIELD_RANGES = {
    "SOF": (0, 1),
    "ID": (1, 12),
    "RTR": (12, 13),
    "IDE": (13, 14),
    "EDL": (14, 15),
    "r0": (15, 16),
    "BRS": (16, 17),
    "ESI": (17, 18),
    "DLC": (18, 22),
    "DATA": (22, 22 + 64),  # max 64 bits
    "CRC": (86, 101),
    "CRC_DELIM": (101, 102),
    "ACK": (102, 103),
    "ACK_DELIM": (103, 104),
    "EOF": (104, 111)
}

class VcdViewer(QDialog):
    def __init__(self, vcd_path):
        super().__init__()
        self.setWindowTitle(f"VCD Viewer - {os.path.basename(vcd_path)}")
        self.resize(1200, 800)

        self.layout = QVBoxLayout(self)

        # Create matplotlib figure with two subplots
        self.canvas = FigureCanvas(Figure(figsize=(14, 2.5)))
        self.toolbar = NavigationToolbar(self.canvas, self)
        self.layout.addWidget(self.toolbar)
        self.layout.addWidget(self.canvas)

        # Create subplot for CAN packet structure
        self.ax1 = self.canvas.figure.add_subplot(111)

        # Add table and packet info side-by-side
        self.table_info_layout = QHBoxLayout()

        # Table for CAN fields
        self.table = QTableWidget()
        self.table.setMinimumWidth(self.width() // 2)
        self.table_info_layout.addWidget(self.table, stretch=1)

        # Label for packet info
        self.packet_info_label = QLabel("Packet Information")
        self.packet_info_label.setWordWrap(True)
        self.packet_info_label.setMinimumWidth(self.width() // 2)
        self.table_info_layout.addWidget(self.packet_info_label, stretch=1)

        # Add to main layout
        self.layout.addLayout(self.table_info_layout)


        # Navigation controls
        self.button_layout = QHBoxLayout()
        self.prev_button = QPushButton("Previous")
        self.next_button = QPushButton("Next")
        self.channel_button = QPushButton("Channel: 0")
        self.packet_label = QLabel("Packet: 0/0")
        self.button_layout.addWidget(self.prev_button)
        self.button_layout.addWidget(self.packet_label)
        self.button_layout.addWidget(self.channel_button)
        self.button_layout.addWidget(self.next_button)
        self.layout.addLayout(self.button_layout)

        # Connect signals
        self.prev_button.clicked.connect(self._prev_packet)
        self.next_button.clicked.connect(self._next_packet)
        self.channel_button.clicked.connect(self._change_channel)

        # Initialize variables
        self.json_data = {}
        self.packets = {}
        self.signal_data = {}
        self.packet_times = []
        self.current_index = 0
        self.channel = 0

        # Load JSON data for CAN ID configuration
        json_path = os.path.join(os.path.dirname(__file__), "CAN_ID.json")
        self._parse_json_data(json_path)

        # Load and display data
        self._parse_and_plot(vcd_path)

    def _parse_json_data(self, filename):
        try:
            with open(filename, "r") as f:
                raw = json.load(f)
                can_id_data = raw.get("can_id", {})
                # Convert keys to integers for easier lookup
                self.json_data = {int(k): v for k, v in can_id_data.items()}
        except Exception as e:
            print(f"Error loading CAN ID config: {e}")


    def _parse_and_plot(self, path):
        signals = {}
        current_time = 0
        current_packet = []
        packet_start_time = None

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
                        self.signal_data[name] = []

                elif line.startswith('#'):
                    try:
                        current_time = int(line[1:])
                        if packet_start_time is None:
                            packet_start_time = current_time
                    except ValueError:
                        pass

                elif line[0] in ('0', '1'):
                    value = int(line[0])
                    code = line[1:]
                    name = signals.get(code, code)
                    self.signal_data[name].append((current_time, value))

        # Convert signal data to binary representation
        bit_delay_fast = 4000
        bit_delay_send = 4112
        bitlines = {name: [] for name in self.signal_data.keys()}
        for name, data in self.signal_data.items():
            prev_time = 0
            prev_value = None
            bits = ""
            for time, value in data:
                
                if name == "D2" and prev_time and prev_time == 0 and time != 0:
                    time = 0
                
                transmision_time = (time - prev_time)
                #print(f"Transmision Time: {name} {transmision_time}")
                prev_time = time
                # if transmision_time > 10000000:
                #     continue
                if name in ["D2", "D3"]:
                    bits_no = transmision_time // bit_delay_send
                else:
                    bits_no = transmision_time // bit_delay_fast
                if prev_value is not None:
                    bits += str(prev_value) * bits_no
                prev_value = value

            bitlines[name] = bits
        
        
        # Separate bits into packets
        no_of_signals = len(self.signal_data.keys()) // 2
        self.packets = {signal: [] for signal in range(no_of_signals)}
        current_packet = ""

        # Search for first 0 bit on second signal and check if the bit on the first is 1
        for signal in range(no_of_signals):
            bits_h = bitlines.get(f'D{signal * 2}')
            bits_l = bitlines.get(f'D{signal * 2 + 1}')

            bit_counter = 0
            packet_check = False
            for bit_h, bit_l in zip(bits_h, bits_l):
                if bit_h == '1' and bit_l == '0':
                    # Start a new packet
                    if bit_counter != 0:
                        bit_counter += 1
                        current_packet += bit_l
                        continue
                    current_packet = "0"
                    bit_counter = 1
                    packet_check = True
                elif bit_counter > 162:
                    # End of packet
                    self.packets[signal].append(current_packet)
                    bit_counter = 0
                    packet_check = False
                elif packet_check:
                    # Add bits to current packet
                    current_packet += bit_l
                    bit_counter += 1

        # Corect packets based on frequency
        for signal in range(no_of_signals):
            corrected_packets = []
            for packet in self.packets[signal]:
                frame = ""
                # Reconstruct the frame based on BRS
                if packet[32] == '1':  # normal rate
                    # Extract every other bit starting from the first bit
                    frame = ''.join([packet[i] for i in range(0, len(packet), 2)])
                else:  # BRS is 0, so we read at the faster rate
                    aux = ''.join([packet[i] for i in range(0, 34, 2)])
                    #print(f"Aux: {aux}")
                    frame += aux
                    dlc = int(''.join([packet[i] for i in range(35, 39)]), 2)
                    data_length = dlc * 8
                    #print(f"DLC: {dlc}, Data Length: {data_length}")
                    aux = packet[34:34 + 1 + 4 + data_length + 17]
                    #print(f"Aux: {aux}")
                    frame += aux
                    aux = ''.join([packet[i] for i in range(34 + 4 + 1 + data_length + 17, 34 + 4 + 1 + data_length + 17 + 2 + 2 + 2 + 14, 2)])
                    #print(f"Aux: {aux}")
                    #print(f"     {frame[:-(1 + 4 + 17 + data_length)]}")
                    frame += aux
                    #frame += "1" * (81 - len(frame))  # Fill remaining bits with '1's
                    if frame[40 + data_length] == '1':
                        print("ACK AAAA " + packet + " " + frame)
                        # Ignotre this packet
                        continue
                corrected_packets.append(frame[:81])  # Ensure we only keep the first 81 bits
            self.packets[signal] = corrected_packets

        # Filter packets based on presence in JSON data
        for signal in list(self.packets.keys()):
            filtered = []
            for packet in self.packets[signal]:
                try:
                    if len(packet) < 22:
                        continue  # skip malformed

                    id_bits = packet[1:12]  # ID field bits (bit positions 1 to 11)
                    packet_id = int(id_bits, 2)

                    if packet_id in self.json_data:
                        filtered.append(packet)
                except Exception as e:
                    print(f"Packet filtering error: {e}")
                    continue
            self.packets[signal] = filtered

        # Show first packet
        if self.packets[0]:
            self._show_packet(0)
        else:
            self.ax1.text(0.5, 0.5, 'No packets found',
                         horizontalalignment='center',
                         verticalalignment='center',
                         transform=self.ax1.transAxes)
            self.canvas.draw()

    def _show_packet(self, index):
        if index >= len(self.packets[self.channel]):
            return

        # Clear plot
        self.ax1.clear()

        # Get current packet
        packet = self.packets[self.channel][index]
        bin_val = packet.strip()

        # Plot CAN packet structure
        self._plot_binary_representation(bin_val)

        # Show CAN fields
        self._show_fields(bin_val)

        # Update packet counter
        self.packet_label.setText(f"Packet: {index + 1}/{len(self.packets[self.channel])}")
        self.canvas.draw()


    def _plot_binary_representation(self, bin_val):
        self.ax1.set_ylim(-0.5, 3.5)
        self.ax1.set_yticks([0, 1, 2, 3])
        self.ax1.set_yticklabels(['CAN_L 0', 'CAN_L 1', 'CAN_H 0', 'CAN_H 1'])
        self.ax1.grid(True, axis='x', linestyle='--', alpha=0.5)

        total_bits = len(bin_val)
        total_width = 0
        bit_positions = []
        current_pos = 0

        # First: compute field lengths (like before)
        field_lengths = []
        for field_name, field_length in CAN_FIELDS:
            if callable(field_length):
                if field_name == "DATA":
                    dlc_bits = bin_val[18:22]
                    try:
                        dlc = int(dlc_bits, 2)
                        actual_length = dlc * 8
                    except ValueError:
                        actual_length = 0
                else:
                    actual_length = 0
            else:
                actual_length = field_length
            field_lengths.append(actual_length)

        # Calculate bit positions
        for i, (field_name, _) in enumerate(CAN_FIELDS):
            field_length = field_lengths[i]
            if field_name not in ["ESI", "DLC", "DATA", "CRC"]:
                for i in range(field_length):
                    bit_positions.append(current_pos + i * 2)
                current_pos += field_length * 2
            else:
                for i in range(field_length):
                    bit_positions.append(current_pos + i)
                current_pos += field_length
            total_width = max(total_width, current_pos)

        bit_positions += [total_width]
        self.ax1.set_xlim(-1, total_width)
        
        while len(bit_positions) <= len(bin_val):
            bit_positions.append(bit_positions[-1] + 1)

        # Plot both CAN_L and CAN_H
        prev_can_l = None
        prev_can_h = None

        for i, bit in enumerate(bin_val):
            can_l = int(bit)
            can_h = 1 - can_l
            pos = bit_positions[i]
            next_pos = bit_positions[i + 1] if i < total_bits - 1 else total_width

            # CAN_L plot
            if prev_can_l is not None and prev_can_l != can_l:
                self.ax1.plot([pos, pos], [prev_can_l, can_l], 'r-', linewidth=1.5)
            self.ax1.plot([pos, next_pos], [can_l, can_l], 'b-', linewidth=1.2)
            prev_can_l = can_l

            # CAN_H plot (shifted to y + 2)
            if prev_can_h is not None and prev_can_h != can_h:
                self.ax1.plot([pos, pos], [prev_can_h + 2, can_h + 2], 'g-', linewidth=1.5)
            self.ax1.plot([pos, next_pos], [can_h + 2, can_h + 2], 'm-', linewidth=1.2)
            prev_can_h = can_h

            if i < total_bits - 1:
                self.ax1.axvline(x=next_pos, color='gray', linestyle='-', alpha=0.2)

        # Field labels/rectangles (unchanged)
        current_pos = 0
        for i, (field_name, _) in enumerate(CAN_FIELDS):
            field_length = field_lengths[i]
            if not isinstance(field_length, int) or field_length <= 0:
                continue

            start_pos = bit_positions[current_pos]
            if field_name == "EOF":
                end_pos = total_width
            elif i < len(CAN_FIELDS) - 1:
                next_field_length = field_lengths[i + 1]
                if isinstance(next_field_length, int) and next_field_length > 0:
                    end_pos = bit_positions[current_pos + field_length]
                else:
                    end_pos = bit_positions[current_pos + field_length - 1] + 1
            else:
                end_pos = bit_positions[current_pos + field_length - 1] + 1

            color = FIELD_COLORS.get(field_name, 'lightgray')
            rect = patches.Rectangle((start_pos, -0.5), end_pos - start_pos, 4,
                                    facecolor=color, alpha=0.2, edgecolor='black')
            self.ax1.add_patch(rect)
            label_y = 3.25 if (i % 2 == 0) else -0.35
            self.ax1.text((start_pos + end_pos) / 2, label_y, field_name,
                        ha='center', va='bottom', fontsize=8)

            current_pos += field_length

        self.ax1.set_title("CAN_L and Mirrored CAN_H Binary Representation", fontsize=12)
        self.ax1.set_xlabel("Bit Position", fontsize=10)


    def _show_fields(self, bin_val):
        try:
            if not isinstance(bin_val, str) or not all(c in '01' for c in bin_val):
                raise ValueError("Invalid binary value input")

            self.table.setRowCount(len(CAN_FIELDS))
            self.table.setColumnCount(4)
            self.table.setHorizontalHeaderLabels(["Field", "Binary", "Decimal", "Hex"])

            current_pos = 0
            for row, (field_name, field_length) in enumerate(CAN_FIELDS):
                try:
                    # Determine actual field length
                    if callable(field_length):
                        if field_name == "DATA":
                            if len(bin_val) < 22:
                                raise IndexError("bin_val too short for DLC")
                            dlc_bits = bin_val[18:22]
                            dlc = int(dlc_bits, 2)
                            actual_length = dlc * 8
                        else:
                            actual_length = 0
                    else:
                        actual_length = field_length

                    if current_pos + actual_length > len(bin_val):
                        field_binary = bin_val[current_pos:] + '0' * (current_pos + actual_length - len(bin_val))
                    else:
                        field_binary = bin_val[current_pos:current_pos + actual_length]

                    decimal_value = int(field_binary or "0", 2)
                    hex_value = format(decimal_value, f'0{actual_length // 4 + (1 if actual_length % 4 else 0)}x')

                except Exception as e:
                    decimal_value = 0
                    hex_value = '0'
                    field_binary = ''
                    print(f"Error parsing field '{field_name}': {e}")

                if field_name == "ID":
                    try:
                        info_data = self.json_data.get(decimal_value, "Unknown")

                        if isinstance(info_data, dict):
                            pretty_info = json.dumps(info_data, indent=2)
                        else:
                            pretty_info = str(info_data)

                        def colorize_json(json_str):
                            import re
                            json_str = re.sub(r'"(.*?)":', r'<span style="color:#d73a49;">"\1"</span>:', json_str)
                            json_str = re.sub(r': "(.*?)"', r': <span style="color:#032f62;">"\1"</span>', json_str)
                            json_str = re.sub(r': (\d+)', r': <span style="color:#005cc5;">\1</span>', json_str)
                            return json_str

                        pretty_info_html = colorize_json(pretty_info)

                        self.packet_info_label.setText(f"""
                        <pre style="
                            background-color: #f0f0f0;
                            border: 1px solid #ccc;
                            padding: 10px;
                            font-family: Courier New, monospace;
                            font-size: 12pt;
                            white-space: pre-wrap;
                        ">{pretty_info_html}</pre>
                        """)
                    except Exception as e:
                        print(f"Error rendering ID info: {e}")
                        self.packet_info_label.setText("<i>Error displaying packet info.</i>")

                try:
                    items = [
                        QTableWidgetItem(field_name),
                        QTableWidgetItem(field_binary),
                        QTableWidgetItem(str(decimal_value)),
                        QTableWidgetItem(hex_value)
                    ]

                    bg_color = QColor(FIELD_COLORS.get(field_name, "#FFFFFF"))
                    for col, item in enumerate(items):
                        item.setBackground(bg_color)
                        self.table.setItem(row, col, item)
                except Exception as e:
                    print(f"Error updating table row {row}: {e}")

                current_pos += actual_length

            self.table.resizeColumnsToContents()
            self.table.setMinimumHeight(300)

        except Exception as e:
            print(f"Fatal error in _show_fields: {e}")
            self.packet_info_label.setText("<i>Unable to display fields due to internal error.</i>")

   

    def _prev_packet(self):
        if self.current_index > 0:
            self.current_index -= 1
            self._show_packet(self.current_index)
        else:
            self.current_index = len(self.packets[self.channel]) - 1
            self._show_packet(self.current_index)

    def _next_packet(self):
        if self.current_index < len(self.packets[self.channel]) - 1:
            self.current_index += 1
            self._show_packet(self.current_index)
        else:
            self.current_index = 0
            self._show_packet(self.current_index)

    def _change_channel(self):
        self.channel = (self.channel + 1) % len(self.packets)
        self.current_index = 0
        self.channel_button.setText(f"Channel: {self.channel}")
        self._show_packet(self.current_index)



if __name__ == '__main__':
    import sys
    import os
    from PyQt5.QtWidgets import QApplication, QFileDialog

    app = QApplication(sys.argv)

    # Check for -path=... argument
    file_path = None
    for arg in sys.argv[1:]:
        if arg.startswith("-path="):
            file_path = arg[len("-path="):]
            # Optional: Normalize path for safety
            file_path = os.path.abspath(file_path)
            break

    # If no path was provided via command-line, open file dialog
    if not file_path:
        file_path, _ = QFileDialog.getOpenFileName(None, "Deschide fișier VCD", "", "VCD Files (*.vcd)")

    # If a path is available (either from cmd or dialog), open it
    if file_path:
        viewer = VcdViewer(file_path)
        viewer.show()
        sys.exit(app.exec_())
    else:
        print("No file selected")
        sys.exit(0)
