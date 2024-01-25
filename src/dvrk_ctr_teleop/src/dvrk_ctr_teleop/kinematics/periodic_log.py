import rospy
import threading

class PeriodicLogger(threading.Thread):
    def __init__(self, hz):
        self.database = {}
        r = rospy.Rate(hz)
        self.done = False
        self.start()

    def set(self, name, value):
        self.database[name] = value

    def stop(self):
        self.done = True
        self.join()

    def run(self):
        while not self.done and not rospy.is_shutdown():
            for item in self.database.items():
                print(item)
            self.r.sleep()
