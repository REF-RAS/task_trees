# Copyright 2024 - Andrew Kwok Fai LUI, 
# Robotics and Autonomous Systems Group, REF, RI
# and the Queensland University of Technology

__author__ = 'Andrew Lui'
__copyright__ = 'Copyright 2024'
__license__ = 'GPL'
__version__ = '1.0'
__email__ = 'ak.lui@qut.edu.au'
__status__ = 'Development'

from collections import defaultdict

class StepInfo():
    """ The supporting class models the information of a step in scanning over a region
    """
    TODO = 0
    DONE = 1
    def __init__(self, ix:int, iy:int, xy:list):
        self.ix, self.iy = ix, iy
        self.xy = xy
        self.state = StepInfo.TODO

class ScanRegionModel():
    """ The class for modelling the physical poses of the steps for the ScanTask, which scans over a region for pick-up objects
    """
    def __init__(self, region_bbox: list, z_level:float, step_size: list):
        # setup the scanning sequence
        self.current_index = 0
        self.z_level = z_level
        self.scangrid = defaultdict(lambda: None)
        self.scanseq = []

    def set_state(self, ix:int, iy:int, state:int):
        if (ix, iy) in self.scangrid:
            self.scangrid[ix, iy].state = state

    def done_current(self):
        self.scanseq[self.current_index].state = StepInfo.DONE
        self.current_index += 1
        
    def get_next_xyz(self) -> list:
        if self.current_index >= len(self.scanseq):
            return None
        return self.scanseq[self.current_index].xy + [self.z_level]
    
    def reset(self):
        for step in self.scanseq:
            step.state = StepInfo.TODO
        self.current_index = 0

class SingleLineScanModel(ScanRegionModel):
    """ The class defines a scanning pattern of a single line
    """
    def __init__(self, region_bbox: list, z_level:float, step_size: list):   
        super(SingleLineScanModel, self).__init__(region_bbox, z_level, step_size)

        x, y = region_bbox[0], region_bbox[1]
        ix, iy = 0, 0
        self.scanseq.append(StepInfo(ix, iy, [x, y]))
        self.scangrid[ix, iy] = self.scanseq[-1]
        x, y = region_bbox[2], region_bbox[3]
        ix, iy = 1, 0
        self.scanseq.append(StepInfo(ix, iy, [x, y]))
        self.scangrid[ix, iy] = self.scanseq[-1]        

class SteppingScanModel(ScanRegionModel):
    """ The class defines a scanning pattern of a single line
    """
    def __init__(self, region_bbox: list, z_level:float, step_size: list):   
        super(SteppingScanModel, self).__init__(region_bbox, z_level, step_size)

        x, y = region_bbox[0], region_bbox[1]
        ix, iy = 0, 0
        while True:
            if x >= region_bbox[2]:
                x, ix = region_bbox[0], 0
                y += step_size[1]
                iy += 1
            if y > region_bbox[3]:
                break
            self.scanseq.append(StepInfo(ix, iy, [x, y]))
            self.scangrid[ix, iy] = self.scanseq[-1]
            ix += 1
            x += step_size[0]
        # initialize the progress
        self.current_index = 0

class FourCornersScanModel(ScanRegionModel):
    """ The class defines a scanning pattern of a single line
    """
    def __init__(self, region_bbox: list, z_level:float, step_size: list):   
        super(FourCornersScanModel, self).__init__(region_bbox, z_level, step_size)

        x, y = region_bbox[0], region_bbox[1]
        ix, iy = 0, 0
        self.scanseq.append(StepInfo(ix, iy, [x, y]))
        self.scangrid[ix, iy] = self.scanseq[-1]
        x, y = region_bbox[2], region_bbox[1]
        ix, iy = 1, 0
        self.scanseq.append(StepInfo(ix, iy, [x, y]))
        self.scangrid[ix, iy] = self.scanseq[-1]  
        x, y = region_bbox[2], region_bbox[3]
        ix, iy = 1, 1
        self.scanseq.append(StepInfo(ix, iy, [x, y]))
        self.scangrid[ix, iy] = self.scanseq[-1] 
        x, y = region_bbox[0], region_bbox[3]
        ix, iy = 1, 0
        self.scanseq.append(StepInfo(ix, iy, [x, y]))
        self.scangrid[ix, iy] = self.scanseq[-1] 