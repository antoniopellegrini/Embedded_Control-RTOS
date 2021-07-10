import tkinter as tk
from telemetry import telemetry
from New_turtle import Plot
from tkinter import *
import threading
import pandas as pd
from scipy.stats import zscore
import numpy as np
import time
from tkinter import ttk

class Application(tk.Frame):

    def print_contents(self, event):
        #TODO: text entry not working properly
        if self.status:
            print("Command to sent:", self.contents.get())

    def Take_input(self, event):
        #INPUT = inputtxt.get("1.0", "end-1c")
        print("Taking Input")

        self.tele_data = self.t.communicate(self.contents.get())
        self.Output.insert(END, self.tele_data)

        print(f" DATIII {self.tele_data}")


        #self.Output.insert(END, )
        #self.Output.insert(END, "ask")

    def Send_cmd_0(self):

        # INPUT = inputtxt.get("1.0", "end-1c")
        print("Taking Input")

        self.tele_data_2 = self.t.communicate(0)
        self.Output.insert(END, self.tele_data_2)

        print(f" DATIII {self.tele_data_2}")

    def Send_cmd_1(self):
        # INPUT = inputtxt.get("1.0", "end-1c")
        print("Taking Input")

        self.tele_data_2 = self.t.communicate(1)
        self.Output.insert(END, self.tele_data_2)

        print(f" DATIII {self.tele_data_2}")

    def Send_cmd_2(self):
        # INPUT = inputtxt.get("1.0", "end-1c")
        print("Taking Input")

        self.tele_data_2 = self.t.communicate(2)
        self.Output.insert(END, self.tele_data_2)

        print(f" DATIII {self.tele_data_2}")



    def AskData(self):

        while True:
            #print("asking data")
            if self.t.isportopen():
                self.tele_data = self.t.communicate('')
                self.Output.insert(END,self.tele_data )
                #time.sleep(0.1)

                self.lista = str(self.tele_data).split(";")

                if len(self.lista) == 6:

                    try:

                        if self.lista[0].split(":")[1].strip() == "2":
                            self.P1_direction = float(self.lista[4].split(',')[0])
                            self.P2_direction = float(self.lista[5].split('.')[0])
                            self.B1_direction = (self.P1_direction + self.P2_direction) / 2

                        elif self.lista[0].split(":")[1].strip() == '3':
                            self.P1_direction = float(self.lista[4].split(',')[0])
                            self.P2_direction = float(self.lista[5].split('.')[0])
                            self.B2_direction = (self.P1_direction + self.P2_direction)/2

                    except:
                        print("can't translate")

                    self.time = self.lista[2]
                    self.direction = float(self.lista[3])


                    print(f"Obtained data: {self.B1_direction}  {self.B2_direction}  {self.direction}")
                    self.is_started = True


                    #self.Graph_update()

                    #self.g.update(self.B1_direction, self.B2_direction, self.direction)


                    #except:
                    #    print("not readable")

            self.Output.see('end')


    def AskData_OLD(self):

        print("askdata start")
        if self.t.isportopen():
            print("asking data")
            #app.after(10, self.Output.insert(END, self.t.communicate('')))
            self.Output.insert(END, self.t.communicate(''))



    def __init__(self, master=None):
        self.status = "Init"
        super().__init__(master)
        self.master = master
        self.pack()
        self.create_widgets()
        self.t = telemetry()

        self.is_started = False
        self.time = 0
        self.direction = 0.0
        self.B1_direction = 0.0
        self.B2_direction = 0.0
        self.P1_direction = 0.0
        self.P2_direction = 0.0

        self.quit_graph = False

        root.geometry("500x500")
        root.title(" Rocket Control GUI ")

    def Clear(self):
        self.Output.delete('1.0', END)

    def Graph(self):
        self.g = Plot()

    def Graph_update(self):

        self.Graph()
        while True:

            if self.quit_graph:
                self.g.quit_window()

            self.g.update(self.B1_direction, self.B2_direction, self.direction, self.time, self.is_started)

    def Plotting(self):

        def clean(direction):
            dir = df[[direction]].values
            diffs = [np.abs(dir[i] - dir[i + 1]) for i in range(len(dir) - 1)]
            dir_cleaned = np.full(dir.shape, np.nan)  # create a array full of NaNs
            dir_cleaned[0] = dir[0]  # The first value isn't a outlier

            for i in range(1, len(dir)):    # remove outliers
                if np.abs(dir[i] - dir[i - 1]) < np.std(diffs) * 30  and dir[i] < 1.6 * np.mean(dir):
                    dir_cleaned[i] = dir[i]

            return dir_cleaned

        df = pd.read_csv('Log_file.txt')
        dir_1 = clean('dir1')
        dir_2 = clean('dir2')

        #df2 = pd.DataFrame(df, columns=['direction', 'dir1', 'dir2', 'time'])
        df2 = pd.DataFrame(df, columns=['direction', 'time'])
        merge = pd.concat([df2, pd.DataFrame(dir_1, columns=['dir_1_clean']),
                           pd.DataFrame(dir_2, columns=['dir_2_clean'])], axis=1)
        # pd.DataFrame({'dir_1_clean': list(dir_1)})
        merge.plot(x='time', use_index=True, ylabel="degree", grid=True)


    def QuitAll(self):
        self.Plotting()
        self.quit_graph = True
        self.master.destroy()

    def Connect(self):

        def update2(status):

            def state(st):
                self.status = st
                print(self.label["text"])
                self.label["text"] = f"State:{self.status}"

            if status:
                self.open.configure(state=ACTIVE)
                self.close.configure(state=ACTIVE)
                self.btn_0.configure(state=ACTIVE)
                self.btn_1.configure(state=ACTIVE)
                self.btn_2.configure(state=ACTIVE)
                self.entrythingy.configure(state="normal")
                #self.entrythingy.bind('<Key-Return>', self.print_contents)
                state("Connected!")


                #app.after(10, app.AskData())
                #self.Take_input()

            else:
                self.open.configure(state=DISABLED)
                self.close.configure(state=DISABLED)
                self.entrythingy.configure(state=DISABLED)
                self.btn_0.configure(state=DISABLED)
                self.btn_1.configure(state=DISABLED)
                self.btn_2.configure(state=DISABLED)
                #self.entrythingy.bind("<Key>", lambda e: "break")  # Disable characters from keyboard
                state("NOT Connected!")

        if self.t.connect():
            update2(True)

            thread = threading.Thread(target=self.AskData)
            thread.start()

            thread = threading.Thread(target=self.Graph_update())
            thread.start()

            #app.after(10, self.AskData())
        else:
            update2(False)

    def isportopen(self):
        self.t.isportopen()

    def Open(self):
        self.t.open()
        #self.Take_input()

    def Close(self):
        self.t.close()
        self.g.clear_window()
        self.btn_0.configure(state=DISABLED)
        self.btn_1.configure(state=DISABLED)
        self.btn_2.configure(state=DISABLED)




    def create_widgets(self):

        # Create label
        self.label = Label(root, text=f"State:{self.status}")
        self.label.config(font=("Courier", 14))
        self.label.pack()

        # COMMAND LINE
        self.entrythingy = tk.Entry(root, state='disabled')
        self.entrythingy.pack(side="top")
        # Create the application variable.
        self.contents = tk.StringVar()
        # Set it to some value.
        self.contents.set("")
        # Tell the entry widget to watch this variable.
        self.entrythingy["textvariable"] = self.contents
        self.entrythingy.bind('<Key-Return>',  self.Take_input)# self.print_contents)

        # TEXT BOX
        #self.Output = Text(root, height=10, width=25, bg="light cyan")
        #self.Output.pack()
        self.Output = Text(root, height=10, width=25, bg="light cyan")
        self.vsb = tk.Scrollbar(root, orient="vertical", command=self.Output.yview)
        self.Output.configure(yscrollcommand=self.vsb.set)
        self.vsb.pack(side="right", fill="y")
        self.Output.pack(side="left", fill="both", expand=True)
        self.Output.see('end')



        self.clear_txt = tk.Button(self, text="Clear", fg="green", command=self.Clear)
        self.clear_txt.pack(side='right')

        self.connect = tk.Button(self, text="Connect", fg="green", command=self.Connect)
        self.connect.pack( side='right')

        self.open = tk.Button(self, text="Open", fg="green", command=self.Open)
        self.open.configure(state=DISABLED)
        self.open.pack( side='right')

        self.close = tk.Button(self, text="Close", fg="green", command=self.Close)
        self.close.configure(state=DISABLED)
        self.close.pack(side='right')

        self.quit = tk.Button(self, text="QUIT", fg="red", command=self.QuitAll)
        self.quit.pack(side="right")

        self.btn_0 = tk.Button(self, text="0-New_Cmd", fg="green", command=self.Send_cmd_0 )
        self.btn_0.configure(state=DISABLED)
        self.btn_0.pack(side='top')

        self.btn_1 = tk.Button(self, text="1-Calibrate", fg="green", command=self.Send_cmd_1)
        self.btn_1.configure(state=DISABLED)
        self.btn_1.pack(side='top')

        self.btn_2 = tk.Button(self, text="2-Start", fg="green", command=self.Send_cmd_2)
        self.btn_2.configure(state=DISABLED)
        self.btn_2.pack(side='top')


    def say_hi(self):
        print("hi there, everyone!")



root = tk.Tk()
app = Application(master=root)

#root.after(2000, app)

app.mainloop()
