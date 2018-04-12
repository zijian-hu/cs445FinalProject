"""
Helper classes to process a simplified vector image file (YAML format)
The file supports bezier curves and lines. Both are generated from a
given SVG path string, as documented here: https://www.w3.org/TR/SVG/paths.html.

Look at the lab11_plot.py file to find an example usage of the classes defined here.
"""
import math
import numpy as np
import re
import yaml


def bezier(P, t):
    return (1-t)**3 * P[0] + 3*(1-t)**2*t*P[1] + 3*(1-t)*t**2*P[2]+t**3 * P[3]


def dbezier(P, t):
    return 3*(1-t)**2 * (P[1] - P[0]) + 6 * (1-t) * t * (P[2] - P[1]) + 3 * t**2 * (P[3]-P[2])


class BezierPath:
    def __init__(self, path, color="black", translate=[0,0], scale=1.0):
        self.beziers = self.parse_path(path)
        self.color = color
        self.translate = translate
        self.scale = scale

    def eval(self, segment, t):
        # frac, whole = math.modf(t)
        # P = self.beziers[int(whole)]
        P = self.beziers[segment]
        pos = bezier(P, t)
        pos2 = bezier(P, t+0.01)
        heading = math.atan2(-(pos2[1]-pos[1]), pos2[0]-pos[0])
        # dr = dbezier(b, t)
        # heading = math.degrees(math.atan2(-dr[1], dr[0]))
        return np.array([(pos[0]+self.translate[0])*self.scale, (-pos[1]+self.translate[1]) * self.scale, heading])

    def num_segments(self):
        return len(self.beziers)

    def get_arc_length(self, i):
        ts = np.linspace(0, 1.0)
        result = 0
        last_state = None
        for t in ts[:-2]:
            state = self.eval(i, t)
            if last_state is not None:
                result += np.linalg.norm(state[0:2] - last_state[0:2])
            last_state = state
        return result

    def get_total_arc_length(self):
        result = 0
        for i in range(0, len(self.beziers)):
            result += get_arc_length(i)
        return result

    def get_start(self):
        return self.eval(0, 0)

    def get_end(self):
        return self.eval(self.num_segments()-1, self.get_length())

    def parse_path(self, path):
        result = []
        pos = None
        while len(path) > 0:
            if path.startswith("m "):
                pattern = "^m (?P<x>[-+]?\d*\.?\d+),(?P<y>[-+]?\d*\.?\d+)\s(?P<rest>.*)$"
                r = re.findall(pattern, path)
                pos = np.array([float(r[0][0]), float(r[0][1])])
                path = r[0][2]
            elif path.startswith("c "):
                pattern = "^c (?P<cmd>[-+\d\.,\s]*)(?P<rest>.*)$"
                r = re.findall(pattern, path)
                cmd = r[0][0].split(" ")
                currentBezier = [pos]
                for c in cmd:
                    pattern2 = "^(?P<x>[-+]?\d*\.?\d+),(?P<y>[-+]?\d*\.?\d+)$"
                    n = re.findall(pattern2, c)
                    if len(n) > 0:
                        x = float(n[0][0])
                        y = float(n[0][1])
                        currentBezier.append(np.array([x + pos[0], y + pos[1]]))
                        if len(currentBezier) == 4:
                            result.append(currentBezier)
                            pos = bezier(currentBezier, 1)
                            currentBezier = [pos]
                path = r[0][1]
            elif path.startswith("C "):
                pattern = "^C (?P<cmd>[-+\d\.,\s]*)(?P<rest>.*)$"
                r = re.findall(pattern, path)
                cmd = r[0][0].split(" ")
                currentBezier = [pos]
                for c in cmd:
                    pattern2 = "^(?P<x>[-+]?\d*\.?\d+),(?P<y>[-+]?\d*\.?\d+)$"
                    n = re.findall(pattern2, c)
                    if len(n) > 0:
                        x = float(n[0][0])
                        y = float(n[0][1])
                        currentBezier.append(np.array([x, y]))
                        if len(currentBezier) == 4:
                            result.append(currentBezier)
                            pos = bezier(currentBezier, 1)
                            currentBezier = [pos]
                path = r[0][1]
            else:
                raise "Couldn't parse!"
        return result


class Line:
    def __init__(self, u, v, color="black", translate=[0,0], scale=1.0):
        self.color = color
        translate = np.array(translate)
        self.u = (np.array([u[0], -u[1]]) + translate) * scale
        self.v = (np.array([v[0], -v[1]]) + translate) * scale
        self.color = color

def parse_lines(lines):
    result = []
    pos = np.array([0,0])
    while len(lines) > 0:
        if lines.startswith("m "):
            pattern = "^m (?P<cmd>[-+\d\.,\s]*)(?P<rest>.*)$"
            r = re.findall(pattern, lines)
            cmd = r[0][0].split(" ")
            for c in cmd:
                pattern2 = "^(?P<x>[-+]?\d*\.?\d+),(?P<y>[-+]?\d*\.?\d+)$"
                n = re.findall(pattern2, c)
                if len(n) > 0:
                    x = float(n[0][0])
                    y = float(n[0][1])
                    # if pos is None:
                    #     result.append(np.array([x, y]))
                    #     pos = result[-1]
                    # else:
                    result.append(np.array([x + pos[0], y + pos[1]]))
                    pos = result[-1]
            lines = r[0][1]
        else:
            raise "Couldn't parse!"
    return result


class VectorImage:
    def __init__(self, file_name):
        with open(file_name, 'r') as f:
            file = yaml.load(f)
        self.paths = []
        self.lines = []
        for p in file["paths"]:
            self.paths.append(BezierPath(p["data"], p["color"], file["translate"], file["scale"]))
        for l in file["lines"]:
            points = parse_lines(l["data"])
            for i in range(0, len(points)-1):
                self.lines.append(Line(points[i], points[i+1], l["color"], file["translate"], file["scale"]))
