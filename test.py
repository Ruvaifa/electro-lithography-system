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
shift_coordinates("parallel.txt", "parallel_large.txt")

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

    print(f"[✅] Generated {len(coords)} points across {num_lines} parallel lines.")
    print(f"[💾] Saved to '{output_file}'")

# Example usage
# generate_parallel_lines(
#    num_lines=6,
#    line_length=40000,      # microns or any units
#    line_spacing=4000,      # spacing between lines
#    point_spacing=100# spacing between any two points
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



