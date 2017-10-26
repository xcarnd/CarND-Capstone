import rospy
import signal
import time
import sys
import matplotlib.pyplot as plt
from std_msgs.msg import String

class Debug_visual():
    def __init__(self, argv):
        self.control_frequency = rospy.get_param('control_frequency', 30)

        self.all_msg = []
        self.legends = []
        print len(argv)
        self.draw_type = 'line' if len(argv) < 2 else argv[1]
        self.max_save_size = 5000 if len(argv) < 3 else argv[2]
        self.vision_size = 1500 if len(argv) < 4 else argv[3]
        self.color = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 
                        'tab:purple', 'tab:brown', 'tab:pink', 'tab:gray', 'tab:olive', 'tab:cyan']
        self.maker = ['.', '-o', '*', '+', ',']
        self.map_key_x = '_x'
        self.map_key_y = '_y'
        plt.ion()

    def init_legend(self, legends):
        for key in legends:
            is_x = self.map_key_x in key
            is_y = self.map_key_y in key
            key = key.strip('\"\' ')

            if (self.draw_type == 'map' and (is_x or is_y)) \
                or (self.draw_type == 'line' and not( is_x or is_y)):
                self.legends.append(key)
            else:
                self.legends.append('')

    def debug_cb(self, msg):
        """
            msg is a str consist of 2 arrays. e.g: '[['velocity', 'angle'],[10.2, 0.6]]'
        """
        keys = msg.data.strip('[ ]').split('],')[0].strip('[ ]').split(',')
        msg = msg.data.strip('[ ]').split('],')[1].strip('[ ]').split(',')
        # init
        if self.all_msg == []:
            self.all_msg = [ [] for i in range(len(msg))]
            self.init_legend(keys)
   
        for i in range(len(msg)):
            if self.legends[i] == '':
                continue
            self.all_msg[i].append(float(msg[i]))
            if str(self.max_save_size).isdigit():
                self.all_msg[i] = self.all_msg[i][-self.max_save_size:]

    def visual(self):
        rate = rospy.Rate(self.control_frequency)
        while len(self.all_msg) == 0: 
            rate.sleep()        

        while not rospy.is_shutdown():
            if 'map' == self.draw_type:
                self.draw_2d_map()
            elif 'line' == self.draw_type:
                self.draw_line()

            rate.sleep()
            plt.pause(0.05)
            plt.clf()
       
    def draw_line(self, maker='.', markersize=2):
        """
            This function draw points without explicit x value.
        """
        for i in range(len(self.legends)):
            is_x = self.map_key_x in self.legends[i]
            is_y = self.map_key_y in self.legends[i]
            is_empty = self.legends[i] == ''
  
            if is_x or is_y or is_empty:
                continue  

            y = self.all_msg[i]
            c = i%len(self.color)
            if str(self.vision_size).isdigit():
                x_start  = max(0, len(y) - self.vision_size + 200)
                plt.xlim(x_start, x_start + self.vision_size)
            plt.plot(y, maker, markersize=markersize, color=self.color[c], label=self.legends[i])
        legends = [i for i in self.legends if i !='' ]
        plt.legend(legends, numpoints=4)

    def draw_2d_map(self, maker='.', markersize=2):
        """
            This function draw points with explicit (x, y).
            If the label contain `self.keyword_2d_map`,  means that the label are x values.
            For example self.keyword_2d_map = '_x':
                I'll draw point only if the legends contain words like "target_x" and "target".
        """
        legends = []
        for i in range(len(self.legends)):
            if self.map_key_x not in self.legends[i]:
                continue
            # If legens[i] is x, find its y and plot.
            label = self.legends[i][:-len(self.map_key_x)]
            label_y = label + self.map_key_y
            y_pos = [j for j,v in enumerate(self.legends) if v==label_y]

            if y_pos == []:
                continue

            x = self.all_msg[i]
            y = self.all_msg[y_pos[0]]
            c = i%len(self.color)
            if str(self.vision_size).isdigit(): 
                t = int(self.vision_size*0.2)
                x_start = x[-self.vision_size] if len(x) > self.vision_size else x[0]
                y_start = y[-self.vision_size] if len(y) > self.vision_size else y[0]
                x_end = x[-1] + (x[-1] - x[-t])  if len(x)>t else x[-1]
                y_end = y[-1] + (y[-1] - y[-t])  if len(y)>t else y[-1]

                plt.xlim(x_start, x_end)
                plt.ylim(y_start, y_end)
            legends.append(label)
            plt.plot(x, y, maker, color=self.color[c], markersize=markersize, label=self.legends[i])
        
        plt.legend(legends, numpoints=4)     

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
    debug = Debug_visual(sys.argv)
    debug.listerner()