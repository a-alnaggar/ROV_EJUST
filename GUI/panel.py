import tkinter as tk

# Create the main window
root = tk.Tk()

# Set the title and size of the window
root.title("Rectangular Panel")
root.geometry("400x300")

# Create a frame to hold the rectangular panel
panel_frame = tk.Frame(root, width=300, height=200, bg="lightgray")
panel_frame.pack(pady=20)

# Create labels to display the values
depth_label = tk.Label(panel_frame, text="Depth: ")
depth_label.grid(row=0, column=0, padx=10, pady=10)
depth_value = tk.Label(panel_frame, text="10")
depth_value.grid(row=0, column=1, padx=10, pady=10)

diameter_label = tk.Label(panel_frame, text="Diameter: ")
diameter_label.grid(row=1, column=0, padx=10, pady=10)
diameter_value = tk.Label(panel_frame, text="5")
diameter_value.grid(row=1, column=1, padx=10, pady=10)

height_label = tk.Label(panel_frame, text="Height: ")
height_label.grid(row=2, column=0, padx=10, pady=10)
height_value = tk.Label(panel_frame, text="20")
height_value.grid(row=2, column=1, padx=10, pady=10)

total_area_label = tk.Label(panel_frame, text="Total Deceased Area: ")
total_area_label.grid(row=3, column=0, padx=10, pady=10)
total_area_value = tk.Label(panel_frame, text="1000")
total_area_value.grid(row=3, column=1, padx=10, pady=10)

# Create a label to display a photo
photo_label = tk.Label(panel_frame, image=None)
photo_label.grid(row=0, column=2, rowspan=4, padx=10, pady=10)

# # Load and display a sample photo
# photo = tk.PhotoImage(file="sample.png")
# photo_label.config(image=photo)
# photo_label.image = photo

# Start the GUI event loop
root.mainloop()