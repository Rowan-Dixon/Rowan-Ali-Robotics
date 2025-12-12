def print_map(pose_est, rbpf_slam, map_display):
    # Get probability map from SLAM
    prob_map = rbpf_slam.get_probability_map()
    height_cells = len(prob_map)
    width_cells = len(prob_map[0]) if height_cells > 0 else 0

    # Clear the display (white background)
    disp_w = map_display.getWidth()
    disp_h = map_display.getHeight()
    cell_scale_x = disp_w / width_cells
    cell_scale_y = disp_h / height_cells
    cell_scale = min(cell_scale_x, cell_scale_y) #scales with display size
    map_display.setColor(0xFFFFFF)  # white
    map_display.fillRectangle(0, 0, disp_w, disp_h)

    # Draw each cell
    for j in range(height_cells):
        for i in range(width_cells):
            p = prob_map[j][i]

            # Color by occupancy probability
            if p < 0.3:
                color = 0xFFFFFF  # free -> white
            elif p > 0.7:
                color = 0x000000  # occupied -> black
            else:
                color = 0x808080  # unknown-ish -> grey

            map_display.setColor(color)

            # Convert map cell (i, j) to display coordinates
            x_screen = i * cell_scale
            # invert y so top of map is top of display
            y_screen = (height_cells - 1 - j) * cell_scale

            map_display.fillRectangle(x_screen, y_screen,
                                        cell_scale, cell_scale)
    # could we add the actual robot overlayed ontop of the map here?