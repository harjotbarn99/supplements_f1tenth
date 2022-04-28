import rosbag
import rospy
from rosbag.bag import BagMessage
from rospy import Time


class Bag:
    def __init__(self, path, reference_topic, sub_topics, time_skip_millisec) -> None:
        self.bag = rosbag.Bag(path)
        self.reference_topic = reference_topic
        self.sub_topics = sub_topics

        self.time_skip_millisec = time_skip_millisec
        self.time_skip_sec = time_skip_millisec/1000

        self.topics = sub_topics+[reference_topic]
        self.all_topics_in_bag = []

        self.generators = {}
        self.prepare_generators()

        self.current_ref = next(self.generators[self.reference_topic], None)
        self.current_sub = self.update_sub_topics()

    def has_next(self):
        if self.current_ref == None or self.current_sub == None:
            return False
        return True

    def next(self):
        ret = self.current_sub
        ret[self.reference_topic] = self.current_ref
        self.update_observations()
        return ret

    def update_observations(self):
        self.update_ref_topic()
        self.update_sub_topics()
        return

    def prepare_generators(self):
        for t in self.topics:
            self.generators[t] = self.bag.read_messages(topics=[t])
        return

    def update_ref_topic(self):
        self.current_ref = self.get_first_msg_after(self.current_ref, self.generators[self.reference_topic], self.time_skip_sec)
        return
    
    def update_sub_topics(self):
        mp = {}
        for t in self.sub_topics:
            mp[t]=self.get_first_msg_after(self.current_ref,self.generators[t])
        self.current_sub = mp
        return mp
    
    def get_first_msg_after(self,msg:BagMessage, gen,offset_sec=0):
        time = msg.timestamp.to_sec() + offset_sec
        found = False
        while not found:
            cur = next(gen,None)
            if cur == None:
                return None
            if cur.timestamp.to_sec() > time:
                return cur
        return None
    
    def get_all_topics(self):
        if self.all_topics_in_bag != []:
            return self.all_topics_in_bag
        for topic, _, _ in self.bag.read_messages():
            if topic not in self.all_topics_in_bag:
                self.all_topics_in_bag.append(topic)
        return self.all_topics_in_bag
    
    def get_all_topics_in_txt(self):
        with open("topics_in_bag.txt","w") as w:
            for t in self.get_all_topics():
                w.write(t+"\n")
        return

    
def pr(mp):
    for i, j in mp.items():
        print(j.timestamp.to_sec(),"   ->    " ,i)
    print("\n")
    return



if __name__ == "__main__":
    bag = Bag("../data/test.bag","/scan",["/vesc/odom"],100)
    print(bag.has_next())
    for i in range(1000):
        bag.next()
    for i in range(5):
        pr(bag.next())
    

    