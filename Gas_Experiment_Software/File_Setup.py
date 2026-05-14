import csv
from pathlib import Path
from datetime import datetime
from tkinter import filedialog


def get_save_path(parent=None):
    """Prompt user for a filename. Pass in root window if calling from a GUI."""
    file_path = filedialog.asksaveasfilename(
        parent=parent,
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv")],
        title="Save experiment data as..."
    )

    if not file_path:
        return None

    p = Path(file_path)
    date_str = datetime.now().strftime("%Y-%m-%d")
    new_filename = f"{date_str}_{p.stem}{p.suffix}"
    return p.parent / new_filename


def init_csv(path, headers):
    """Create the CSV file and write the header row."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(headers)

def write_csv(path, row):
    """Append a single row of data to the CSV file."""
    with open(path, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(row)