Header header
int32 traj_id
int32 order  # polynomial order, always 5 for MINCO

# Piece durations
float64[] duration

# Polynomial coefficients for each piece
# Each piece has 6 coefficients (order 5)
float64[] coef_x
float64[] coef_y
float64[] coef_z

time start_time

