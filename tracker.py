class Tracker:
    def __init__(self, create_tracker, tag_id, sd_x=0, sd_y=0, sd_theta=0, rate=0):
        self.tag_id = tag_id
        self.sd_x = sd_x
        self.sd_y = sd_y
        self.sd_theta = sd_theta
        self.rate = rate
        self.create_tracker = create_tracker(tag_id, sd_x=sd_x, sd_y=sd_y, sd_theta=sd_theta, rate=rate)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self):
            r = self.create_tracker.query()
            if r is not None:
                self.x = r["position"]["x"]
                self.y = r["position"]["y"]
                self.theta = r["orientation"]["y"]
