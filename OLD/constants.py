TABLE_STYLE = """
    QTableWidget {
        border: 1px solid #dee2e6;
        border-radius: 4px;
        background: white;
    }
    QHeaderView::section {
        background: #f8f9fa;
        padding: 5px;
        border: 1px solid #dee2e6;
        font-weight: bold;
        font-size: 12px;
        min-height: 30px;
    }
    QTableWidget::item {
        padding: 2px;
        font-size: 10px;
    }
    QTableWidget::item:selected {
        background: #007bff;
        color: white;
    }
"""
HEADERS = ["ID", "Data", "Source", "Destination", "Name", "Level", "Type", "Period", "Data\nSize", "CARLA Var"]
WIDTHS = [30, 80, 60, 90, 100, 80, 60, 50, 30, 80]
COLOR_MAP_BY_CAN_ID = {
    "26":  "#e74c3c",
    "47":  "#3498db",
    "88":  "#1abc9c",
    "109": "#f39c12",
    "440": "#c0392b",
    "457": "#b71540",
    "131": "#9b59b6",
    "423": "#f1c40f",
    "433": "#e67e22",
    "1200": "#ff5e57",
    "2100": "#95a5a6",
    "1313": "#00cec9"
}

CAN_FIELDS = [
    ("SOF", 1), ("ID", 11), ("R1", 1), ("IDE", 1), ("EDL", 1),
    ("r0", 1), ("BRS", 1), ("ESI", 1), ("DLC", 4),
    ("DATA", lambda x: x * 8), ("CRC", 17), ("CRC_DELIM", 1),
    ("ACK", 1), ("ACK_DELIM", 1), ("EOF", 7)
]

FIELD_COLORS = {
    "SOF": "#AED6F1", "ID": "#F5B7B1", "R1": "#D2B4DE",
    "IDE": "#A9DFBF", "EDL": "#F9E79F", "r0": "#FADBD8",
    "BRS": "#D5F5E3", "ESI": "#D6EAF8", "DLC": "#FCF3CF",
    "DATA": "#F5CBA7", "CRC": "#E8DAEF", "CRC_DELIM": "#D4EFDF",
    "ACK": "#F5CBA7", "ACK_DELIM": "#E8DAEF", "EOF": "#D4EFDF"
}
