# import matplotlib.pyplot as plt

# def plot_e_like_snake():
#     x_start = 10000
#     y_start = 10000
#     line_length = 4000  # Horizontal segment length
#     snake_path = []

#     groups = [
#         (25, 4),   # (vertical drop, number of lines)
#         (50, 4),
#         (100, 4)
#     ]

#     direction = 1  # 1 = left to right, -1 = right to left
#     current_x = x_start
#     current_y = y_start

#     for spacing, count in groups:
#         for _ in range(count):
#             # Draw horizontal line
#             next_x = current_x + direction * line_length
#             snake_path.append((current_x, current_y))
#             snake_path.append((next_x, current_y))
#             current_x = next_x
#             current_y += spacing  # Drop to next line
#             snake_path.append((current_x, current_y))
#             direction *= -1  # Reverse direction

#     # Remove last vertical drop to keep shape clean
#     snake_path.pop()

#     # Plotting
#     xs = [pt[0] for pt in snake_path]
#     ys = [pt[1] for pt in snake_path]

#     fig, ax = plt.subplots(figsize=(10, 6))
#     ax.plot(xs, ys, color='red', linewidth=2, marker='o')
#     ax.set_title("E-Shaped Snake Path (Horizontal Zigzag with Drops)")
#     ax.set_xlabel("X (microns)")
#     ax.set_ylabel("Y (microns)")
#     ax.set_aspect('equal')
#     ax.grid(True)
#     plt.show()

# plot_e_like_snake()


# def generate_e_like_snake_coordinates(filename="e_snake_path.txt"):
#     x_start = 10000
#     y_start = 10000
#     line_length = 5000  # Horizontal segment length
#     snake_path = []

#     groups = [
#         (25, 3),   # (vertical spacing, number of lines)
#         (50, 3),
#         (100, 3)
#     ]

#     direction = 1  # 1 = left to right, -1 = right to left
#     current_x = x_start
#     current_y = y_start

#     for spacing, count in groups:
#         for _ in range(count):
#             next_x = current_x + direction * line_length
#             snake_path.append((current_x, current_y))
#             snake_path.append((next_x, current_y))
#             current_x = next_x
#             current_y += spacing
#             snake_path.append((current_x, current_y))
#             direction *= -1

#     # Remove last drop
#     snake_path.pop()

#     # Save to file
#     with open(filename, 'w') as f:
#         for x, y in snake_path:
#             f.write(f"{x} {y}\n")

#     print(f"Coordinates saved to {filename}")

# generate_e_like_snake_coordinates()
##


##POINTS PER LINE
# def generate_e_like_snake_coordinates(filename="e_snake_path.txt", points_per_line=100):
#     x_start = 10000
#     y_start = 10000
#     line_length = 5000  # Horizontal segment length
#     snake_path = []

#     groups = [
#         (25, 3),   # (vertical spacing, number of lines)
#         (50, 3),
#         (100, 3)
#     ]

#     direction = 1  # 1 = left to right, -1 = right to left
#     current_x = x_start
#     current_y = y_start

#     for spacing, count in groups:
#         for _ in range(count):
#             next_x = current_x + direction * line_length

#             # Generate interpolated horizontal line
#             for i in range(points_per_line):
#                 t = i / (points_per_line - 1)
#                 x = int(current_x + t * (next_x - current_x))
#                 y = current_y
#                 snake_path.append((x, y))

#             # Drop down vertically
#             current_x = next_x
#             current_y += spacing
#             snake_path.append((current_x, current_y))
#             direction *= -1

#     # Remove last drop
#     snake_path.pop()

#     # Save to file
#     with open(filename, 'w') as f:
#         for x, y in snake_path:
#             f.write(f"{x} {y}\n")

#     print(f"Coordinates with {points_per_line} points per line saved to {filename}")

# generate_e_like_snake_coordinates()
########################################################

## DISTANCE BETWEEN TWO POINTS
def generate_e_like_snake_coordinates(filename="lines_100.txt", point_spacing=2.5):
    x_start = 10000
    y_start = 10000
    line_length = 1000  # Horizontal segment length in microns
    snake_path = []

    groups = [
        (25, 0),   # (vertical spacing, number of lines)
        (50, 0),
        (100, 2)
    ]

    direction = 1  # 1 = left to right, -1 = right to left
    current_x = x_start
    current_y = y_start

    for spacing, count in groups:
        for _ in range(count):
            # Horizontal line segment
            next_x = current_x + direction * line_length
            horizontal_length = abs(next_x - current_x)
            horizontal_points = max(2, int(horizontal_length // point_spacing) + 1)

            for i in range(horizontal_points):
                t = i / (horizontal_points - 1)
                x = int(current_x + t * (next_x - current_x))
                y = current_y
                snake_path.append((x, y))

            # Vertical segment (drop down)
            prev_y = current_y
            current_x = next_x
            current_y += spacing
            vertical_length = abs(current_y - prev_y)
            vertical_points = max(2, int(vertical_length // point_spacing) + 1)

            for i in range(1, vertical_points):  # Start from 1 to avoid duplicate point
                t = i / (vertical_points - 1)
                x = current_x
                y = int(prev_y + t * (current_y - prev_y))
                snake_path.append((x, y))

            direction *= -1

    # Save to file
    with open(filename, 'w') as f:
        for x, y in snake_path:
            f.write(f"{x} {y}\n")

    print(f"Coordinates saved to {filename} with spacing every {point_spacing} microns")

# Example usage:
#generate_e_like_snake_coordinates(point_spacing=2)

#-------------------------------------------------------------------------------------------
#ADD BUFFER TO COODINATES
def shift_coordinates(input_file, output_file, buffer=10000):
    with open(input_file, 'r') as infile, open(output_file, 'w') as outfile:
        for line in infile:
            parts = line.strip().replace(',', ' ').split()
            if len(parts) >= 3:
                x, y, flag = float(parts[0]), float(parts[1]), parts[2]
                x += buffer
                y += buffer
                outfile.write(f"{x:.3f}, {y:.3f}, {flag}\n")
    print("done")
# Example usage
# shift_coordinates("parallel_var.txt", "parallel_var.txt.txt")

##------------------------------------------------------------------------------------------------------------------------------------------
#GENERATE PARALLEL LINES
def generate_parallel_lines(
    num_lines,
    line_length,
    line_spacing,
    point_spacing,
    start_x=0,
    start_y=0,
    output_file="parallel.txt"
):
    coords = []
    for i in range(num_lines):
        y = start_y + i * line_spacing
        x = start_x
        while x <= start_x + line_length:
            coords.append((round(x, 3), round(y, 3)))
            x += point_spacing

    # Save to text file
    with open(output_file, "w") as f:
        for x, y in coords:
            f.write(f"{x}, {y}\n")

    print(f"[SUCCESS] Generated {len(coords)} points across {num_lines} parallel lines.")
    print(f"[SAVED] Saved to '{output_file}'")

# Example usage
# generate_parallel_lines(
#    num_lines=6,
#    line_length=40000,      # microns or any units
#    line_spacing=4000,      # spacing between lines
#    point_spacing=100 # spacing between any two points
# )

#---------------------------------------------------------------------------------
# Generate 5 parallel lines with non-uniform spacing between each line

def generate_parallel_lines_variable_spacing(
    line_length,
    point_spacing,
    line_offsets,
    start_x=10000,
    start_y=10000,
    output_file="parallel_variable_spacing.txt"
):
    coords = []
    for offset in line_offsets:
        y = start_y + offset
        x = start_x
        while x <= start_x + line_length:
            coords.append((round(x, 3), round(y, 3)))
            x += point_spacing

    # Save to text file
    with open(output_file, "w") as f:
        for x, y in coords:
            f.write(f"{x}, {y}\n")

    print(f"[SUCCESS] Generated {len(coords)} points across {len(line_offsets)} parallel lines.")
    print(f"[SAVED] Saved to '{output_file}'")

# Example usage (commented out)
# generate_parallel_lines_variable_spacing(
#    line_length=1000,
#    point_spacing=1,
#    line_offsets=[0, 200, 600, 1100, 1700],
#    start_x=10000,
#    start_y=10000
# )

#---------------------------------------------------------------------------------
#GENERATE HORIZONTAL AND VERTICAL LINES
def generate_grid_lines_to_file(n_horizontal, n_vertical, line_length, point_spacing, line_spacing, filename="coordinates.txt"):
    coordinates = []

    # Generate horizontal lines with spacing in Y direction
    for i in range(n_horizontal):
        y = i * line_spacing
        for x in range(0, line_length + 1, point_spacing):
            coordinates.append((x, y))

    # Generate vertical lines with spacing in X direction
    for i in range(n_vertical):
        x = i * line_spacing
        for y in range(0, line_length + 1, point_spacing):
            coordinates.append((x, y))

    # Write to file
    with open(filename, "w") as f:
        for x, y in coordinates:
            f.write(f"{x}, {y}\n")

    print(f"[INFO] {len(coordinates)} coordinates written to '{filename}'")

# Example usage
n_horizontal = 2     # Number of horizontal lines
n_vertical = 2       # Number of vertical lines
line_length = 1000   # Length of each line (in microns or units)
point_spacing = 1    # Distance between data points on each line
line_spacing = 200   # Distance between two lines

#generate_grid_lines_to_file(n_horizontal, n_vertical, line_length, point_spacing, line_spacing)


#-------------------------------------------------
import math
import csv

def generate_circle_coordinates(
    radius=20000,
    center_x=25000,
    center_y=25000,
    num_points=1000,
    output_file="circle_relative.csv"
):
    # Generate absolute circle coordinates
    coords = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        coords.append((x, y))
    
    # Convert to relative coordinates (differences), rounded
    relative_coords = []
    prev_x, prev_y = coords[0]
    for x, y in coords[1:]:
        dx = round(x - prev_x)
        dy = round(y - prev_y)
        relative_coords.append((dx, dy))
        prev_x, prev_y = x, y

    # Close the circle (last point back to start)
    dx = round(coords[0][0] - prev_x)
    dy = round(coords[0][1] - prev_y)
    relative_coords.append((dx, dy))
    
    # Save to CSV
    with open(output_file, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["dx", "dy"])
        writer.writerows(relative_coords)
    
    print(f"[SUCCESS] Generated {len(relative_coords)} relative coordinates for circle.")
    print(f"[SAVED] Saved to '{output_file}'")

# Example usage
#generate_circle_coordinates()

#---------------------------------------------------------------------------------
# GENERATE CONCENTRIC HEXAGONS WITH LIFTOFF FLAGS
def generate_concentric_hexagons(
    radii=[1000, 2000, 3000],
    center_x=10000,
    center_y=10000,
    output_file="concentric_hexagons.txt"
):
    coords = []
    
    for idx, r in enumerate(radii):
        # Generate the 6 vertices of the hexagon + 1 closing point
        hex_points = []
        for k in range(7):
            angle = k * (2 * math.pi / 6)
            x = center_x + r * math.cos(angle)
            y = center_y + r * math.sin(angle)
            hex_points.append((round(x, 3), round(y, 3)))
            
        # Draw this hexagon (flag = 0)
        # For the first hexagon, draw all 7 points.
        # For subsequent hexagons, the touchdown transition point (appended below)
        # already represents the start of drawing (k=0, flag=0), so we draw from k=1
        # onwards to avoid duplicate consecutive coordinates.
        start_point_idx = 0 if idx == 0 else 1
        for x, y in hex_points[start_point_idx:]:
            coords.append((x, y, 0))
            
        # Transition to next hexagon if not the last one
        if idx < len(radii) - 1:
            end_x, end_y, _ = coords[-1]
            
            # 1. Lift off at end of current hexagon: flag = 1
            coords.append((end_x, end_y, 1))
            
            # 2. Start point of next hexagon
            next_r = radii[idx + 1]
            next_start_x = round(center_x + next_r * math.cos(0), 3)
            next_start_y = round(center_y + next_r * math.sin(0), 3)
            
            # 3. Move to start of next hexagon in the air: flag = 1
            coords.append((next_start_x, next_start_y, 1))
            
            # 4. Touch down/re-probe at start of next hexagon: flag = 0
            coords.append((next_start_x, next_start_y, 0))

    # Save to file
    with open(output_file, "w") as f:
        for x, y, flag in coords:
            f.write(f"{x:.3f}, {y:.3f}, {flag}\n")

    print(f"[SUCCESS] Generated {len(coords)} points for {len(radii)} concentric hexagons.")
    print(f"[SAVED] Saved to '{output_file}'")

def plot_concentric_hexagons(filename="concentric_hexagons.txt"):
    """
    Optional visualization of the generated concentric hexagons path using matplotlib.
    Solid lines represent drawing segments (flag=0).
    Dashed lines represent travel/liftoff segments (flag=1).
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[INFO] matplotlib is not installed. Skipping visualization.")
        return

    xs, ys, flags = [], [], []
    try:
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 3:
                    xs.append(float(parts[0]))
                    ys.append(float(parts[1]))
                    flags.append(int(float(parts[2])))
    except Exception as e:
        print(f"[ERROR] Failed to read file for plotting: {e}")
        return

    plt.figure(figsize=(8, 8))
    
    # Plot line segments
    i = 0
    draw_plotted = False
    travel_plotted = False
    while i < len(xs) - 1:
        x1, y1, f1 = xs[i], ys[i], flags[i]
        x2, y2, f2 = xs[i+1], ys[i+1], flags[i+1]
        
        # If either point has liftoff/travel flag (1), it's a travel segment
        if f1 == 1 or f2 == 1:
            label = "Travel/Liftoff (flag=1)" if not travel_plotted else ""
            plt.plot([x1, x2], [y1, y2], color='red', linestyle='--', alpha=0.6, label=label)
            travel_plotted = True
        else:
            label = "Drawing (flag=0)" if not draw_plotted else ""
            plt.plot([x1, x2], [y1, y2], color='blue', linestyle='-', linewidth=2, label=label)
            draw_plotted = True
        i += 1

    # Draw vertices
    plt.scatter(xs, ys, color='black', zorder=5, label='Vertices')
    
    # Annotate points with index and flag
    for idx, (x, y, f) in enumerate(zip(xs, ys, flags)):
        plt.annotate(f"P{idx}(f={f})", (x, y), textcoords="offset points", xytext=(0,8), ha='center', fontsize=7, alpha=0.8)

    plt.title("Concentric Hexagons Patterning Path & Liftoff Signals")
    plt.xlabel("X Coordinate (microns)")
    plt.ylabel("Y Coordinate (microns)")
    plt.grid(True)
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

#---------------------------------------------------------------------------------
# GENERATE CONCENTRIC CIRCLES WITH LIFTOFF FLAGS
def generate_concentric_circles(
    radii=[1000, 2000, 3000],
    center_x=10000,
    center_y=10000,
    point_spacing=100.0,
    output_file="concentric_circles.txt"
):
    """
    Generates coordinates for concentric circles of defined radii.
    Points are computed along each circle's circumference spaced by point_spacing.
    Includes liftoff flags (flag=1) between circles to prevent dragging the probe.
    """
    coords = []
    
    for idx, r in enumerate(radii):
        # Calculate points per circle based on circumference and point_spacing
        circumference = 2 * math.pi * r
        points_per_circle = max(6, int(round(circumference / point_spacing)))
        
        # Generate the vertices of the circle + 1 closing point
        circle_points = []
        for k in range(points_per_circle + 1):
            angle = k * (2 * math.pi / points_per_circle)
            x = center_x + r * math.cos(angle)
            y = center_y + r * math.sin(angle)
            circle_points.append((round(x, 3), round(y, 3)))
            
        # Draw this circle (flag = 0)
        # For the first circle, draw all points.
        # For subsequent circles, the touchdown transition point (appended below)
        # already represents the start of drawing (k=0, flag=0), so we draw from k=1
        # onwards to avoid duplicate consecutive coordinates.
        start_point_idx = 0 if idx == 0 else 1
        for x, y in circle_points[start_point_idx:]:
            coords.append((x, y, 0))
            
        # Transition to next circle if not the last one
        if idx < len(radii) - 1:
            end_x, end_y, _ = coords[-1]
            
            # 1. Lift off at end of current circle: flag = 1
            coords.append((end_x, end_y, 1))
            
            # 2. Start point of next circle
            next_r = radii[idx + 1]
            next_start_x = round(center_x + next_r * math.cos(0), 3)
            next_start_y = round(center_y + next_r * math.sin(0), 3)
            
            # 3. Move to start of next circle in the air: flag = 1
            coords.append((next_start_x, next_start_y, 1))
            
            # 4. Touch down/re-probe at start of next circle: flag = 0
            coords.append((next_start_x, next_start_y, 0))

    # Save to file
    with open(output_file, "w") as f:
        for x, y, flag in coords:
            f.write(f"{x:.3f}, {y:.3f}, {flag}\n")

    print(f"[SUCCESS] Generated {len(coords)} points for {len(radii)} concentric circles.")
    print(f"[SAVED] Saved to '{output_file}'")

def plot_concentric_circles(filename="concentric_circles.txt"):
    """
    Optional visualization of the generated concentric circles path using matplotlib.
    Solid lines represent drawing segments (flag=0).
    Dashed lines represent travel/liftoff segments (flag=1).
    """
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[INFO] matplotlib is not installed. Skipping circle visualization.")
        return

    xs, ys, flags = [], [], []
    try:
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) >= 3:
                    xs.append(float(parts[0]))
                    xs.append(float(parts[1]))
                    flags.append(int(float(parts[2])))
    except Exception as e:
        print(f"[ERROR] Failed to read file for plotting circles: {e}")
        return

    plt.figure(figsize=(8, 8))
    
    # Plot line segments
    i = 0
    draw_plotted = False
    travel_plotted = False
    while i < len(xs) - 1:
        x1, y1, f1 = xs[i], ys[i], flags[i]
        x2, y2, f2 = xs[i+1], ys[i+1], flags[i+1]
        
        # If either point has liftoff/travel flag (1), it's a travel segment
        if f1 == 1 or f2 == 1:
            label = "Travel/Liftoff (flag=1)" if not travel_plotted else ""
            plt.plot([x1, x2], [y1, y2], color='red', linestyle='--', alpha=0.6, label=label)
            travel_plotted = True
        else:
            label = "Drawing (flag=0)" if not draw_plotted else ""
            plt.plot([x1, x2], [y1, y2], color='blue', linestyle='-', linewidth=1.5, label=label)
            draw_plotted = True
        i += 1

    # Draw vertices
    plt.scatter(xs, ys, color='black', s=8, zorder=5, label='Vertices')

    plt.title("Concentric Circles Patterning Path & Liftoff Signals")
    plt.xlabel("X Coordinate (microns)")
    plt.ylabel("Y Coordinate (microns)")
    plt.grid(True)
    plt.legend()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

if __name__ == "__main__":
    # ponytail: Keep entrypoint simple, run both hexagon and circle generators
    
    # Hexagons
    # generate_concentric_hexagons(
    #     radii=[500, 1000, 1500],
    #     center_x=10000,
    #     center_y=10000,
    #     output_file="concentric_hexagons.txt"
    # )
    
    # Circles
    generate_concentric_circles(
        radii=[500, 1000, 1500],
        center_x=10000,
        center_y=10000,
        point_spacing=5.0,
        output_file="concentric_circles.txt"
    )
    
    # Try to plot if matplotlib is available (uncomment to visualize)
    # plot_concentric_hexagons("concentric_hexagons.txt")
    #plot_concentric_circles("concentric_circles.txt")


