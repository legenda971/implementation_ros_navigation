from math import pi
import numpy as np


class Valley:
    """
    Class to represent a valley in a histogram.
    """

    def __init__(self, start, end, size):
        """
        :param start: Integer representing the starting index of the valley.
        :param end: Integer representing the ending index of the valley.
        :param size: Integer representing the size (number of elements) of the valley.
        """
        
        self.start = start
        self.end = end
        self.size = size


class VFH():
    def __init__(self, wide_sectors, sector_num, threshhold, safe_radius, d_max, smoothing, safe_sector):
        """
        :param wide_sectors: Integer representing the minimum number of sectors in a wide valley.
        :param sector_num: Integer representing the number of sectors in the histogram.
        :param threshhold: Float representing the threshold for valley detection.
        :param safe_radius: Float representing the safe radius of the robot, in vfh this variable represent r_s.
        :param safe_sector: Integer representing the safe sector from the edge of a valley in wide valley.
        """
        if 2*safe_sector > wide_sectors:
            raise ValueError("Safe sector must be smaller than wide sectors.")
        
        self.wide_sectors = wide_sectors
        self.sector_num = sector_num
        self.threshhold = threshhold
        self.histogram = None
        self.safe_radius = safe_radius
        self.d_max = d_max
        self.smoothing = smoothing
        self.safe_sector = safe_sector
        
        
    def get_histogram(self):
        return self.histogram
        
    def calculate(self, occupancyGrid, destination_angle):
        """
        
        :param occupancyGrid: OccupancyGrid
        :param destination_angle: float
        
        :return: float = value is angle in radians in format [pi, -pi)
        """
        self.histogram = self.occupancyGrid2Histogram(occupancyGrid)
        self.histogram = self.apply_threshold(self.histogram)
        
        if destination_angle is None:
            return None
        
        valleys = self.calculate_valleys(self.histogram, self.threshhold)
        if valleys is None:
            raise ValueError("No valleys found.")
        
        des_sector = self.angle2sector(destination_angle)
        des_angle = self.sector2angle(des_sector, normalize=True)
        if len(valleys) == 0:
            return des_angle
        
        # for valley in valleys:
        #     if valley.size >= self.wide_sectors and valley.in_valley(des_sector, self.sector_num):
        #         return des_angle
        
        # print("No wide valley found with destination sector in it.")
        # Get closest sector to destination sector
        candidate_sectors = self.get_candidate_sectors(valleys, des_sector)
        
        if len(candidate_sectors) == 0:
            raise ValueError("No candidate sectors found.")
        
        # Sector with minimum distance to destination sector
        best_sector = min(candidate_sectors, key=lambda num: self.sector_score(num, des_sector))
            
        return self.sector2angle(best_sector, normalize=True)
    
    def sector_score(self, sector, destination_sector):
        destination_distance = self.circular_distance(sector, destination_sector)
        yaw_distance = min(self.circular_distance(sector, 0), self.circular_distance(sector, self.sector_num - 1))
        
        return destination_distance + yaw_distance
    
    def circular_distance(self, sector, destination_sector):
        """
        Calculates the minimum distance between two sectors in a circular space with a given total number of sectors.

        :param sector: The sector from which the distance is calculated.
        :param destination_sector: The destination sector to which the distance is calculated.

        :return: Minimum distance between the two sectors.
        """
        distances = [
            abs(sector - destination_sector),
            abs(self.sector_num - sector + destination_sector),
            abs(self.sector_num + sector - destination_sector)
        ]
        return min(distances)

    def sector2angle(self, sector, normalize = False):
        """
        Function to convert a sector number to an angle in radians.
        
        :param sector: int representing the sector number.
        :param normalize: bool indicating whether the angle should be normalized to the range [-pi, pi).
        
        :return: float representing the angle in radians.
        """
        sector_size = 2 * pi / self.sector_num
        result = (sector * sector_size) + (sector_size / 2.0)
        if normalize:
            result = result if result < pi else result - 2 * pi 
        
        return result
    
    
    def angle2sector(self, angle):
        """
        Determines the destination sector for a given angle and number of sectors.

        The function takes an angle (in radians) and divides a full circle into
        'sector_num' equal parts to find the corresponding sector.

        :param angle: The angle in radians. Can be negative, in which case it will be normalized.

        :return Integer representing the sector number.
        """
        while angle < 0:
            angle += 2 * pi
        return int(angle // (2 * pi / self.sector_num))
        
    
    def apply_threshold(self, histogram):
        """
        :param histogram: List[float]
        :param threshold: float
        
        :return: List[float]
        """
        # print("Threshold: ", self.threshhold)
        for i in range(len(histogram)):
            # print(histogram[i], histogram[i] < self.threshhold)
            if histogram[i] < self.threshhold:
                histogram[i] = 0
        return histogram
    
    
    def occupancyGrid2Histogram(self, occupancyGrid):
        """
        Converts an OccupancyGrid to a histogram.
        
        :param occupancyGrid: OccupancyGrid
        
        :return: List[float]
        """                
        width, height = occupancyGrid.info.width, occupancyGrid.info.height
        grid_resolution = occupancyGrid.info.resolution
        angular_resolution = 360.0 / self.sector_num
        
        # Calculate d_max and constants a, b, and c
        d_max = width * grid_resolution / 2
        d_max = min(d_max, self.d_max)
        
        a = d_max  / (d_max - self.safe_radius)
        b = a / d_max
               
        histogram = np.zeros(self.sector_num, dtype=np.float32)
        for i in range(width):
            for j in range(height):
                index = i + j * width
                if occupancyGrid.data[index] == 0:
                    continue

                x = i * grid_resolution - width * grid_resolution / 2.0
                y = j * grid_resolution - height * grid_resolution / 2.0
                
                distance = np.sqrt(x ** 2.0 + y ** 2.0)
                if distance > d_max:
                    continue

                angle = np.rad2deg(np.arctan2(y, x)) % 360

                bin_index = int(angle // angular_resolution)
                histogram[bin_index] += a - b * distance
        
        if self.smoothing > 0:
            histogram = np.convolve(histogram, np.ones(self.smoothing)/self.smoothing, mode='same')
            
        return histogram

    
    def get_candidate_sectors(self, valleys, destination_sector):
        """
        Determines the candidate sectors for navigation based on the given valleys.

        :param valleys: List of Valley objects representing the valleys in the histogram.
        :param destination_sector: Integer representing the sector number of the destination.

        :return: List of integers representing the candidate sectors for navigation.
        """

        # Filter valleys to wide and narrow valleys
        narrow, wide = [], []
        for valley in valleys:
            (wide if valley.size >= self.wide_sectors else narrow).append(valley)
            
        candidate_sectors = []
        for valley in narrow:
            candidate_sectors += self.add_narrow_valley_sector(valley)

        # If destination sector is in middle returned candidate sectors, then return candidate sectors
        for valley in wide:
            temp = self.add_wide_valley_sectors(valley)
            if self.check_if_sector_is_middle_sectors(destination_sector, *temp):
                return [destination_sector]
            else:
                candidate_sectors += temp
            
        return candidate_sectors
    
    def check_if_sector_is_middle_sectors(self, sector, start_sector, end_sector):
        """
        Checks if a sector is between two sectors.
        :param sector: Integer representing the sector number to check.
        :param start_sector: Integer representing the start sector.
        :param end_sector: Integer representing the end sector.
        
        :return: Boolean indicating if the sector is between the start and end sectors.
        """

        if end_sector >= start_sector:
            return start_sector <= sector and sector <= end_sector
        else:
            return (start_sector > sector and sector <= end_sector) or (start_sector <= sector and sector > end_sector)

    def add_wide_valley_sectors(self, valley):
        """
        Adds the sectors for a wide valley to the candidate sectors list.

        :param valley: Valley object representing a wide valley.
        :param candidate_sectors: List of integers to append the candidate sectors to.
        """
        result = []
        result.append((valley.start + self.safe_sector) % self.sector_num)
        result.append((valley.end - self.safe_sector) % self.sector_num)
        
        return result

    def add_narrow_valley_sector(self, valley):
        """
        Adds the sector for a narrow valley to the candidate sectors list. 
        If vallay have even number of sectors, then two sectors are added. 
        If valley have odd number of sectors, then one sector is added.

        :param valley: Valley object representing a narrow valley.
        
        :return candidate_sectors: List of integers to append the candidate sectors to.
        """
        size_mod = self.sector_num if valley.start > valley.end else 0
        midpoint = (valley.start + valley.end + size_mod) // 2

        result = [midpoint % self.sector_num]
        if valley.size % 2 == 0:
            result.append((midpoint + 1) % self.sector_num)

        return result


    def calculate_valleys(self, histogram, threshold):
        """
        Calculates valleys in the given histogram based on a specified threshold.

        A valley is a contiguous segment of the histogram where values are less than or equal to the threshold.
        The function returns a list of Valley objects, each representing a valley in the histogram.

        Special cases:
        1. If the threshold is equal to or greater than all the values in the histogram, the result should be an 
            empty list.
        2. If the threshold is less than all the values in the histogram, the result should be None.

        :param histogram: List of float values representing the histogram.
        :param threshold: Optional float value representing the threshold for valley detection.
                          If not provided, the median of the histogram is used as the threshold.
        :return: Optional list of Valley objects representing the detected valleys.
                 Returns None or an empty list in special cases as described above.
        :raises ValueError: If the histogram is empty or not a list.
        """
        # Validate the histogram
        # if not histogram or not isinstance(histogram, list):
        #     raise ValueError("Invalid histogram input. Expected a non-empty list.")

        # Check if the threshold is less than all the values in the histogram
        if all(value > threshold for value in histogram):
            return None

        # Check if the threshold is greater than or equal to all the values in the histogram
        if all(value <= threshold for value in histogram):
            return []

        valleys = []
        n = len(histogram)
        in_valley = False
        start_idx = 0
        count = 0

        # Iterate through the histogram twice to handle circular nature
        initial_index = None
        for i in range(n * 2):
            idx = i % n

            if initial_index is None:
                if histogram[idx] > threshold:
                    initial_index = idx
                continue

            if histogram[idx] <= threshold:
                if not in_valley:
                    start_idx = idx
                    in_valley = True
                count += 1
            else:
                if in_valley:
                    end_idx = (i - 1) % n
                    valleys.append(Valley(start_idx, end_idx, count))
                    in_valley = False
                    count = 0

            if initial_index == idx:
                break

        return valleys
