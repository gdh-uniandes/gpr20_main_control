

class Planning(object):

    @staticmethod
    def calculate_delta(initial_coord, final_coord, points):
        """Calculate the delta distance for an axis.
        
        Args:
            initial_coord (float): initial coordinate of the axis.
            final_coord (float): final coordinate of the axis.
            points (int): number of points along the axis.

        Returns:
            float: the delta distance of the axis.
        """
        return (final_coord - initial_coord) / (points - 1.0)

    @staticmethod
    def calculate_coord(init_coord, final_coord, indx, delta, direction):
        """Calculates a target coordinate.
        
        
        """
        # Checks if direction is positive
        if direction:
            return init_coord + indx * delta
        
        else:
            return final_coord - indx * delta

