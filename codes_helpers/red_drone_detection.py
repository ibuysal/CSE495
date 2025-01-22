import cv2
import numpy as np
import os
from tkinter import *
from tkinter import ttk
from PIL import Image, ImageTk

class ImageBrowser:
    def __init__(self, root, image_dir):
        self.root = root
        self.root.title("Image Browser with Red Detection")
        
        # Initialize image directory and list
        self.image_dir = image_dir
        self.image_list = [f for f in os.listdir(image_dir) 
                          if f.lower().endswith(('.png', '.jpg', '.jpeg'))]
        self.current_index = 0
        
        # Create main frame
        self.main_frame = ttk.Frame(root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky=(N, W, E, S))
        
        # Create frame for images
        self.images_frame = ttk.Frame(self.main_frame)
        self.images_frame.grid(row=0, column=0, columnspan=4, pady=10)
        
        # Create labels for all images
        self.original_label = ttk.Label(self.images_frame)
        self.original_label.grid(row=0, column=0, padx=5)
        ttk.Label(self.images_frame, text="Original Image").grid(row=1, column=0)
        
        self.mask1_label = ttk.Label(self.images_frame)
        self.mask1_label.grid(row=0, column=1, padx=5)
        ttk.Label(self.images_frame, text="Mask 1 (0-10)").grid(row=1, column=1)
        
        self.mask2_label = ttk.Label(self.images_frame)
        self.mask2_label.grid(row=0, column=2, padx=5)
        ttk.Label(self.images_frame, text="Mask 2 (170-180)").grid(row=1, column=2)
        
        self.combined_label = ttk.Label(self.images_frame)
        self.combined_label.grid(row=0, column=3, padx=5)
        ttk.Label(self.images_frame, text="Combined Result").grid(row=1, column=3)
        
        # Create frames for both red ranges
        self.create_hsv_sliders()
        
        # Create navigation buttons
        self.create_navigation_buttons()
        
        # Load first image
        if self.image_list:
            self.load_image()
            
    def create_hsv_sliders(self):
        # Frame for HSV controls
        hsv_frame = ttk.LabelFrame(self.main_frame, text="Color Mask Controls", padding="5")
        hsv_frame.grid(row=1, column=0, columnspan=3, pady=5, sticky=(W, E))
        
        # First red range (0-10)
        range1_frame = ttk.LabelFrame(hsv_frame, text="Red Range 1 (0-180)", padding="5")
        range1_frame.grid(row=0, column=0, padx=5, pady=5, sticky=(W, E))
        
        ttk.Label(range1_frame, text="Hue Range 1:").grid(row=0, column=0, padx=5)
        self.hue1_low = Scale(range1_frame, from_=0, to=180, orient=HORIZONTAL, command=self.update_mask)
        self.hue1_low.grid(row=0, column=1, padx=5)
        self.hue1_high = Scale(range1_frame, from_=0, to=180, orient=HORIZONTAL, command=self.update_mask)
        self.hue1_high.grid(row=0, column=2, padx=5)
        
        # Second red range (170-180)
        range2_frame = ttk.LabelFrame(hsv_frame, text="Red Range 2 (0-180)", padding="5")
        range2_frame.grid(row=0, column=1, padx=5, pady=5, sticky=(W, E))
        
        ttk.Label(range2_frame, text="Hue Range 2:").grid(row=0, column=0, padx=5)
        self.hue2_low = Scale(range2_frame, from_=0, to=180, orient=HORIZONTAL, command=self.update_mask)
        self.hue2_low.grid(row=0, column=1, padx=5)
        self.hue2_high = Scale(range2_frame, from_=0, to=180, orient=HORIZONTAL, command=self.update_mask)
        self.hue2_high.grid(row=0, column=2, padx=5)
        
        # Common controls frame
        common_frame = ttk.LabelFrame(hsv_frame, text="Common Controls", padding="5")
        common_frame.grid(row=1, column=0, columnspan=2, pady=5, sticky=(W, E))
        
        # Saturation range sliders
        ttk.Label(common_frame, text="Saturation Range:").grid(row=0, column=0, padx=5)
        self.sat_low = Scale(common_frame, from_=0, to=255, orient=HORIZONTAL, command=self.update_mask)
        self.sat_low.grid(row=0, column=1, padx=5)
        self.sat_high = Scale(common_frame, from_=0, to=255, orient=HORIZONTAL, command=self.update_mask)
        self.sat_high.grid(row=0, column=2, padx=5)
        
        # Value range sliders
        ttk.Label(common_frame, text="Value Range:").grid(row=1, column=0, padx=5)
        self.val_low = Scale(common_frame, from_=0, to=255, orient=HORIZONTAL, command=self.update_mask)
        self.val_low.grid(row=1, column=1, padx=5)
        self.val_high = Scale(common_frame, from_=0, to=255, orient=HORIZONTAL, command=self.update_mask)
        self.val_high.grid(row=1, column=2, padx=5)
        
        # Set default values
        self.hue1_low.set(0)
        self.hue1_high.set(10)
        self.hue2_low.set(170)
        self.hue2_high.set(180)
        self.sat_low.set(100)
        self.sat_high.set(255)
        self.val_low.set(100)
        self.val_high.set(255)
        
    def create_navigation_buttons(self):
        # Navigation buttons
        btn_frame = ttk.Frame(self.main_frame)
        btn_frame.grid(row=2, column=0, columnspan=3, pady=10)
        
        ttk.Button(btn_frame, text="Previous", command=self.previous_image).grid(row=0, column=0, padx=5)
        ttk.Button(btn_frame, text="Next", command=self.next_image).grid(row=0, column=1, padx=5)
        
    def load_image(self):
        if not self.image_list:
            return
            
        # Read image
        image_path = os.path.join(self.image_dir, self.image_list[self.current_index])
        self.current_image = cv2.imread(image_path)
        self.update_mask()
        
    def update_mask(self, *args):
        if not hasattr(self, 'current_image'):
            return
            
        # Create copy of original image
        image = self.current_image.copy()
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for both red ranges
        lower1 = np.array([self.hue1_low.get(), self.sat_low.get(), self.val_low.get()])
        upper1 = np.array([self.hue1_high.get(), self.sat_high.get(), self.val_high.get()])
        mask1 = cv2.inRange(hsv, lower1, upper1)

        lower2 = np.array([self.hue2_low.get(), self.sat_low.get(), self.val_low.get()])
        upper2 = np.array([self.hue2_high.get(), self.sat_high.get(), self.val_high.get()])
        mask2 = cv2.inRange(hsv, lower2, upper2)
        
        # Combine masks
        combined_mask = cv2.bitwise_or(mask1, mask2)
        
        # Apply masks to create results
        result1 = cv2.bitwise_and(image, image, mask=mask1)
        result2 = cv2.bitwise_and(image, image, mask=mask2)
        combined_result = cv2.bitwise_and(image, image, mask=combined_mask)
        kernel = np.ones((1,1), np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw bounding boxes on original image
        max_area = 0
        largest_box = None
        
        # First pass: find largest area and draw all boxes in blue
        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            # Draw all boxes in blue
            cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 1)
            
            # Keep track of the largest area
            if area > max_area:
                max_area = area
                largest_box = (x, y, w, h)
        
        # Draw the largest box in red with thicker line
        if largest_box is not None:
            x, y, w, h = largest_box
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # Add area text above the largest box
            cv2.putText(image, f'Largest Area: {int(max_area)}', (x, y-5), 
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        # Convert all images to RGB for display
        original_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        result1_rgb = cv2.cvtColor(result1, cv2.COLOR_BGR2RGB)
        result2_rgb = cv2.cvtColor(result2, cv2.COLOR_BGR2RGB)
        combined_rgb = cv2.cvtColor(combined_result, cv2.COLOR_BGR2RGB)
        
        # Convert to PhotoImage
        display_size = (480, 320)  # Smaller size for four images
        
        # Process original image
        original_pil = Image.fromarray(original_rgb)
        original_pil.thumbnail(display_size, Image.LANCZOS)
        original_photo = ImageTk.PhotoImage(original_pil)
        
        # Process mask1 result
        result1_pil = Image.fromarray(result1_rgb)
        result1_pil.thumbnail(display_size, Image.LANCZOS)
        result1_photo = ImageTk.PhotoImage(result1_pil)
        
        # Process mask2 result
        result2_pil = Image.fromarray(result2_rgb)
        result2_pil.thumbnail(display_size, Image.LANCZOS)
        result2_photo = ImageTk.PhotoImage(result2_pil)
        
        # Process combined result
        combined_pil = Image.fromarray(combined_rgb)
        combined_pil.thumbnail(display_size, Image.LANCZOS)
        combined_photo = ImageTk.PhotoImage(combined_pil)
        
        # Update labels
        self.original_label.configure(image=original_photo)
        self.original_label.image = original_photo
        
        self.mask1_label.configure(image=result1_photo)
        self.mask1_label.image = result1_photo
        
        self.mask2_label.configure(image=result2_photo)
        self.mask2_label.image = result2_photo
        
        self.combined_label.configure(image=combined_photo)
        self.combined_label.image = combined_photo
        
    def next_image(self):
        if self.current_index < len(self.image_list) - 1:
            self.current_index += 1
            self.load_image()
            
    def previous_image(self):
        if self.current_index > 0:
            self.current_index -= 1
            self.load_image()


if __name__ == "__main__":
    root = Tk()
    app = ImageBrowser(root, "/home/ibu/bitirme_ws/noise_20_40_50/150points_3_3_loc_1_0__noise_50_20_7/data/images")
    root.mainloop()
