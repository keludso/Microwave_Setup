import tkinter as tk
from GUI import MFCApp
import u12

if __name__ == "__main__":
    root = tk.Tk()
    d = u12.U12()
    app = MFCApp(root, device=d)
    root.mainloop()
