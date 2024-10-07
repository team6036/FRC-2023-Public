import tkinter as tk
import tkinter.ttk as ttk
import filewriter
import solve
import subprocess

filename = "arm_presets.json"

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        self.tk.call("source", "Azure-ttk-theme-main/azure.tcl")
        self.title("Arm Presets")
        self.positions = filewriter.importData(filename)

        self.tk.call("set_theme", "dark")

        self.selectedPath = tk.StringVar()
        self.selectedPath.set(list(self.positions.keys())[0])
        self.selector = ttk.OptionMenu(self, self.selectedPath, *[list(self.positions.keys())[0]] + list(self.positions.keys()))
        self.selector.pack()

        self.lastSelected = ""

        self.angle1_label = ttk.Label(self, text="Angle 1")
        self.angle1_label.pack(pady=5)
        self.angle1_input = ttk.Entry(self)
        self.angle1_input.pack(pady=5)
        self.angle2_label = ttk.Label(self, text="Angle 2")
        self.angle2_label.pack(pady=5)
        self.angle2_input = ttk.Entry(self)
        self.angle2_input.pack(pady=5)

        self.pasteButton = ttk.Button(self, text="Paste", command=self.paste)
        self.pasteButton.pack(pady=5)


        self.solveButton = ttk.Button(self, text="Solve", command=self.calculate)
        self.solveButton.pack(pady=5)

        self.update()

    def writeData(self):
        filewriter.saveData(filename, self.positions)

    def calculate(self):
        solve.solve()

    def paste(self):
        try:
            p = subprocess.Popen(['pbpaste'], stdout=subprocess.PIPE)
            retcode = p.wait()
            data = str(p.stdout.read()).split("'")[1]
            data = data.split(',')
            self.angle1_input.delete(0, tk.END)
            self.angle1_input.insert(0, data[0])
            self.angle2_input.delete(0, tk.END)
            self.angle2_input.insert(0, data[1])
        except:
            print("Clipboard data invalid")
            return

    def update(self):
        try:
            if self.selectedPath.get() != self.lastSelected:
                self.angle1_input.delete(0, tk.END)
                self.angle1_input.insert(0, self.positions[self.selectedPath.get()][0])
                self.angle2_input.delete(0, tk.END)
                self.angle2_input.insert(0, self.positions[self.selectedPath.get()][1])
                self.lastSelected = self.selectedPath.get()
            if [float(self.angle1_input.get()), float(self.angle2_input.get())] != self.positions[self.selectedPath.get()]:
                self.positions[self.selectedPath.get()] = [float(self.angle1_input.get()), float(self.angle2_input.get())]
                print(self.selectedPath.get(), ":", self.positions[self.selectedPath.get()])
                self.writeData()
        except:
            pass
        finally:
            self.selector.after(500, self.update)
app = App()
app.mainloop()
