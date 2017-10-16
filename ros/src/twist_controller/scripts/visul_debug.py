import rospy
import signal
import time
import matplotlib.pyplot as plt
from std_msgs.msg import String

class Debug_visual(object):
    def __init__(self):
        self.all_msg = []
        self.legends = []
        plt.ion()

    def debug_cb(self, msg):
        """
            msg is a str consist of 2 arrays. e.g: '[['velocity', 'angle'],[10.2, 0.6]]'
        """
        keys = msg.data.strip('[ ]').split('],')[0].strip('[ ]').split(',')
        msg = msg.data.strip('[ ]').split('],')[1].strip('[ ]').split(',')
        # init
        if self.all_msg == []:
            self.all_msg = [ [] for i in range(len(msg))]
            for v in keys:
                self.legends.append(v.strip('\"\' '))

        for i in range(len(msg)):
            self.all_msg[i].append(float(msg[i]))

    def visual(self):
        color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w']
        while True:
            if len(self.all_msg) == 0: 
                time.sleep(0.5)
                continue
            
            if rospy.is_shutdown():
                print "Shutdown, exit."
                break
            for i in range(len(self.legends)):
                pos = i%len(color)
                # # you can use plt.subplot(212) if you need.
                x_start  = max(0, len(self.all_msg[i]) - 80)
                plt.xlim(x_start, x_start + 100)
                plt.plot(self.all_msg[i], color=color[pos], linewidth=1.0, label=self.legends[i])

            plt.legend(self.legends)
            plt.pause(0.1)
       

    def listerner(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.    
        rospy.init_node('listerner', anonymous=True)
        
        rospy.Subscriber("/dbw_node/debug", String, self.debug_cb)

        print 'DBW debug listerner is running.'
        self.visual()
        rospy.spin()
        
def signal_handler(signum, frame):
    print "Exit signal."
    exit(signum)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    debug = Debug_visual()
    debug.listerner()