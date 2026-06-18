# Helper script to adjust bed screws tilt using Z probe

import math
from . import probe

class ScrewsTiltAdjust:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.screws = []
        self.results = {}
        self.max_diff = None
        self.max_diff_error = False
        for i in range(99):
            prefix = "screw%d" % (i + 1,)
            if config.get(prefix, None) is None:
                break
            screw_coord = config.getfloatlist(prefix, count=2)
            # Hard-coded default to prevent "referenced before assignment"
            f_name = config.get(prefix + "_name", "screw_%d" % (i + 1))
            self.screws.append((screw_coord, f_name))
            
        if len(self.screws) < 3:
            raise config.error("screws_tilt_adjust: Must have at least three screws")
            
        self.threads = {'CW-M3': 0, 'CCW-M3': 1, 'CW-M4': 2, 'CCW-M4': 3,
                        'CW-M5': 4, 'CCW-M5': 5, 'CW-M6': 6, 'CCW-M6': 7}
        self.thread = config.getchoice('screw_thread', self.threads, default='CW-M3')
        
        points = [coord for coord, name in self.screws]
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize, default_points=points)
        self.probe_helper.minimum_points(3)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command("SCREWS_TILT_CALCULATE", self.cmd_SCREWS_TILT_CALCULATE)

    def cmd_SCREWS_TILT_CALCULATE(self, gcmd):
        self.max_diff = gcmd.get_float("MAX_DEVIATION", None)
        self.direction = gcmd.get("DIRECTION", default=None)
        self.probe_helper.start_probe(gcmd)

    def get_status(self, eventtime):
        return {'error': self.max_diff_error, 'max_deviation': self.max_diff, 'results': self.results}

    def probe_finalize(self, offsets, positions):
        self.results = {}
        if not positions:
            raise self.gcode.error("SCREWS_TILT_CALCULATE: No data received.")
        
        threads_factor = {0: 0.5, 1: 0.5, 2: 0.7, 3: 0.7, 4: 0.8, 5: 0.8, 6: 1.0, 7: 1.0}
        is_clockwise_thread = (self.thread & 1) == 0
        
        def extract_z(p):
            try:
                if hasattr(p, 'bed_z'): return float(p.bed_z)
                if isinstance(p, (list, tuple)): return float(p[-1]) # Extract last element if it's a coord list
                return float(p)
            except:
                return 0.0

        valid_count = min(len(positions), len(self.screws))
        z_base = extract_z(positions[0])
        
        self.gcode.respond_info("01:20 = 1 turn, 20 mins. CW=Clockwise, CCW=Counter-Clockwise")
        
        screw_diffs = []
        for i in range(valid_count):
            z = extract_z(positions[i])
            coord, name = self.screws[i]
            if i == 0:
                self.gcode.respond_info("%s (base) : z=%.5f" % (name, z))
                self.results["screw%d" % (i + 1,)] = {'z': z, 'adjust': '00:00', 'is_base': True}
            else:
                diff = z_base - z
                screw_diffs.append(abs(diff))
                adj = diff / threads_factor.get(self.thread, 0.5)
                sign = ("CW" if adj >= 0 else "CCW") if is_clockwise_thread else ("CCW" if adj >= 0 else "CW")
                adj = abs(adj)
                turns, mins = math.trunc(adj), round((adj - math.trunc(adj)) * 60, 0)
                self.gcode.respond_info("%s : z=%.5f : %s %02d:%02d" % (name, z, sign, turns, mins))
                self.results["screw%d" % (i + 1,)] = {'z': z, 'adjust': "%02d:%02d" % (turns, mins), 'is_base': False}

def load_config(config):
    return ScrewsTiltAdjust(config)

