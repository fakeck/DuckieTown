import svgwrite

# Parameters for the AprilTag and layout
tag_size = 120  # Size of the AprilTag in mm
square_size = tag_size / 8  # Size of each square in mm (for an 8x8 grid)
boundary_size = 15 # Size of the boundary in mm
base_size = tag_size + 2 * boundary_size  # Size of the AprilTag including the boundary

# Parameters for the layout grid
rows = 1  # Number of rows of AprilTags
cols = 1  # Number of columns of AprilTags
spacing = 20  # Spacing between AprilTags in mm
paper_x_offset = 30
paper_y_offset = 30

outer_color = "black"
inner_color = "#7cbdc5"

tag1_colors = [[0, 0, 0, 0, 0, 0, 0, 0],
               [0, 1, 1, 0, 1, 1, 0, 0],
               [0, 0, 1, 0, 1, 1, 1, 0],
               [0, 1, 1, 1, 1, 0, 0, 0],
               [0, 0, 1, 1, 0, 0, 0, 0],
               [0, 1, 0, 1, 1, 0, 1, 0],
               [0, 0, 0, 1, 0, 0, 1, 0],
               [0, 0, 0, 0, 0, 0, 0, 0]]

tag2_colors = [[0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 1, 0, 1, 1, 1, 0],
                [0, 0, 1, 0, 0, 1, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 1, 0, 0, 1, 0],
                [0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 1, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0]]

tag3_colors = [[0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 1, 1, 0, 0, 1, 0],
                [0, 0, 0, 0, 1, 1, 1, 0],
                [0, 1, 0, 0, 1, 1, 1, 0],
                [0, 1, 0, 1, 0, 0, 1, 0],
                [0, 1, 1, 0, 0, 1, 0, 0],
                [0, 0, 1, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0]]

tag4_colors = [[0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 1, 1, 0, 1, 0, 0],
                [0, 1, 1, 1, 1, 0, 0, 0],
                [0, 1, 0, 1, 1, 1, 1, 0],
                [0, 0, 0, 1, 0, 1, 0, 0],
                [0, 1, 0, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0]]

tag5_colors = [[0, 0, 0, 0, 0, 0, 0, 0],
                [0, 1, 1, 1, 1, 0, 0, 0],
                [0, 1, 1, 0, 0, 0, 1, 0],
                [0, 1, 1, 0, 1, 1, 0, 0],
                [0, 1, 0, 1, 0, 1, 1, 0],
                [0, 0, 0, 1, 1, 1, 0, 0],
                [0, 1, 0, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0]]



# Calculate the total size of the layout grid in mm
total_width = cols * base_size + (cols - 1) * spacing + paper_x_offset
total_height = rows * base_size + (rows - 1) * spacing + paper_y_offset

# Create a new SVG drawing for the entire layout
tag_colors = tag5_colors
dwg = svgwrite.Drawing(f"tag5_blue.svg", profile="tiny", size=("297mm", "210mm"))  # A4 size paper    
# dwg.add(dwg.text(f"tag36h11, ID: {1}", 
#                 insert=(f"{paper_x_offset}mm", f"{paper_y_offset - 15}mm"),
#                 font_size="5mm", font_family="Arial", fill="black"))

# Generate multiple AprilTags and position them within the layout
for row in range(rows):
    for col in range(cols):
        # Get the tag pattern
        

        # Calculate position for each AprilTag within the layout grid
        x_offset = col * (base_size + spacing) + paper_x_offset
        y_offset = row * (base_size + spacing) + paper_y_offset

        # Add the inner boundary rectangle in inner color
        dwg.add(dwg.rect(
            (f"{x_offset - boundary_size}mm", f"{y_offset - boundary_size}mm"),
            (f"{base_size}mm", f"{base_size}mm"),
            fill=inner_color,
            stroke='none'
        ))

        # Create the grid of rectangles (AprilTag pattern)
        for i in range(8):
            y = i * square_size + y_offset
            for j in range(8):
                x = j * square_size + x_offset
                
                # Set color: 0 for red, 1 for green
                color = outer_color if tag_colors[i][j] == 0 else inner_color

                dwg.add(dwg.rect((f"{x}mm", f"{y}mm"), (f"{square_size}mm", f"{square_size}mm"), fill=color, stroke='none'))


# Save the SVG file for the entire layout
dwg.save()
