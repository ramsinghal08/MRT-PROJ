import rclpy
from rclpy.node import Node
from messages.msg import BranchMsg
from messages.msg import InfoMsg
from messages.srv import MoveMembers
from .bot import Bot
import random
from messages.msg import PathMsg
import time
mapping = {}
coordinates_explored = {}  # this will have all the coordinates explored with their branch id as value
branches = []  # this will have all the branch objects in the order in which they are created. Used for retracing so that the child can go to whatever is alive.

# Global universal branch_node instance
_branch_node_instance = None


def get_branch_node():
    
    global _branch_node_instance
    if _branch_node_instance is None:
        if not rclpy.ok():
            raise RuntimeError("rclpy not initialized. Call rclpy.init() first.")
        _branch_node_instance = branch_node()
    return _branch_node_instance


class branch_node(Node):  # The node branch will handle ros related works
    def __init__(self):
        super().__init__("branch_node")
        
        # Dictionary to store all branch objects with their ID as key
        self.dict_branches = {}
        
        self.path_sub = self.create_subscription(PathMsg, "path_topic", self.update_path_callback, 10)
        self.branch_pub = self.create_publisher(BranchMsg, "branch_topic", 10)
        self.move_srv = self.create_service(MoveMembers, 'move_members', self.move_members_callback)
        self.info_sub = self.create_subscription(InfoMsg, "infoNodeTopic", self.callback, 10)
        
        self.get_logger().info("BranchNode initialized")
        self.get_logger().info(f"Subscribed to InfoMsg on topic: infoNodeTopic")
        self.get_logger().info(f"Publishing BranchMsg on topic: branch_topic")

    def add_branch(self, branch_obj):
        
        self.dict_branches[branch_obj.id] = branch_obj
        self.get_logger().info(f"Branch {branch_obj.id} added to node. Total branches: {len(self.dict_branches)}")

    def remove_branch(self, branch_id: int):
        
        if branch_id in self.dict_branches:
            del self.dict_branches[branch_id]
            self.get_logger().info(f"Branch {branch_id} removed from node")

    def get_branch(self, branch_id: int):
        
        return self.dict_branches.get(branch_id, None)

    def callback(self, msg):
        self.get_logger().info(f"Received callback with data: {msg.data}")
        print(msg.c1, msg.c2, msg.c3)
        if tuple(msg.c3) == ():
            surr = []
            surr.append(tuple(msg.c1))
            surr.append(tuple(msg.c2))
            print(surr)
        else:
            surr = []
            surr.append(tuple(msg.c1))
            surr.append(tuple(msg.c2))
            surr.append(tuple(msg.c3))
            print(surr)
        if msg.data == "split":
            self.get_logger().info(f"Branch {msg.branch_id} received split command")
            print(self.dict_branches)
            branch = self.dict_branches.get(msg.branch_id)
            if branch:
                self.get_logger().info(f"Found branch {branch.id}, calling split with {surr}")
                branch.split(surr)
            else:
                self.get_logger().error(f"Branch {msg.branch_id} not found in dict_branches!")

        elif msg.data == "dead-end":
            branch = self.dict_branches.get(msg.branch_id)
            if branch:
                branch.returning()
            else:
                self.get_logger().error(f"Branch {msg.branch_id} not found for dead-end!")

    def publish_branch_info(self, branch):
        msg = BranchMsg()
        self.get_logger().info(f"====== PUBLISHING BRANCH INFO ======")
        print(branch.path)
        self.get_logger().info(f"Publishing info of Branch {branch.id}")
        
        if len(branch.path) > 0:
            if len(branch.path) == 1:
                init = branch.path[0]
            else:
                init = branch.path[-1]
        else:
            init = (0, 0)
        
        msg.init_coord = init
        msg.id = branch.id
        msg.leader_id = branch.leader.id
        print(msg.leader_id)
        msg.splitting = branch.spliting
        msg.member_ids = [i.id for i in branch.members]
        print(msg.member_ids)
        # msg.next_id = branch.prev.id if branch.prev else -1
        # print(msg.next_id)
        self.get_logger().info(f"Branch {branch.id}: leader={branch.leader.id}, init_coord={init}, splitting={branch.spliting}")
        self.get_logger().info(f"Subscriber count on branch_topic: {self.branch_pub.get_subscription_count()}")
        
        self.branch_pub.publish(msg)
        
        self.get_logger().info(f"Branch {branch.id} info published successfully")
        self.get_logger().info(f"====== PUBLISH COMPLETE ======")

    def move_members_callback(self, request, response):
        branch_id = request.branch_id
        target_coord = (request.x, request.y)
        
        if branch_id in self.dict_branches:
            branch = self.dict_branches[branch_id]
            for member in branch.members:
                member.move(target_coord)
            branch.update_path(target_coord)
            response.success = True
        else:
            response.success = False
        return response

    def update_path_callback(self, msg):
        branch = self.dict_branches.get(msg.id)
        if not branch:
            return
        
        coord = msg.coordinate
        
        if coord in coordinates_explored:
            other_branch_id = coordinates_explored[coord]
            if other_branch_id != branch.id:
                other_branch = self.dict_branches.get(other_branch_id)
                if other_branch:
                    self.get_logger().info(f"Branch {branch.id} stepped into Branch {other_branch.id} at coordinate {coord}")
                    branch.update_path(coord)
        else:
            branch.update_path(coord)
            for i in branch.members:
                i.move(coord)
            coordinates_explored[coord] = branch.current


class branch:  # pure python class. This will handle the branch and its function.
    min_size = 2
    i = 1
    
    def __init__(self, members, leader, id=None, spliting=True, retracing=False, path=None, prev=None):
        if id is None:
            self.id = branch.i
            branch.i += 1
        else:
            self.id = id
            
        self.leader = leader
        self.leader.branch_id = self.id
        self.node = get_branch_node()
        self.members = members
        self.leader.set_leader()
        
        for i in self.members:
            i.set_follower()
            i.follow_leader = self.leader
            i.branch_id = self.id
        
        self.path = [] if path is None else path
        self.retracing = retracing
        self.prev = prev
        self.current = self.id
        self.spliting = spliting
        
        # Register this branch with the universal node
        self.node.add_branch(self)
        
        print(f"Branch created with id {self.id}, leader {self.leader.id},members {[i.id for i in self.members]}")
        
        # Publish its info when created
        self.node.publish_branch_info(self)

    def merge_path_based(self, branch_to_merge_into):
        self.members.append(self.leader)
        branch_to_merge_into.update_branch(self.members)
        self.current = branch_to_merge_into.current
        self.node.remove_branch(self.id)

    def split(self, surrounding):
        print(f"Branch with id {self.id} is splitting")
        
        if len(self.members) == 1:
            print(f"branch with id {self.id} is at its min")
            self.spliting = False
            x = random.choice(surrounding)
            new_branch = branch(members=self.members, leader=self.leader, spliting=False, path=[x], id=self.id)
            self.node.remove_branch(self.id)
        else:
            print("split started")
            frontier = surrounding.copy()
            print(frontier)
            new_branch_leader = self.check_leader(self.members)
            print(new_branch_leader.id)
            n = len(self.members) // 2
            
            if len(frontier) == 3:
                print(frontier)
                if len(self.members) < 5:
                    frontier.pop()
                    self.split(frontier)
                else:
                    # choose two new leaders from current members (lowest priority wins)
                    candidates = sorted(self.members, key=lambda b: b.priority)
                    new_leader1 = candidates[0] if len(candidates) >= 1 else new_branch_leader
                    new_leader2 = candidates[1] if len(candidates) >= 2 else (candidates[0] if candidates else new_branch_leader)

                    # remove chosen leaders from the pool of members
                    remaining = [m for m in self.members if m not in (new_leader1, new_leader2)]

                    # evenly distribute remaining members among three groups
                    groups = [[], [], []]
                    for idx, m in enumerate(remaining):
                        groups[idx % 3].append(m)

                    # assign one frontier coordinate to each new branch
                    random.shuffle(frontier)
                    coord_a, coord_b, coord_c = frontier[0], frontier[1], frontier[2]

                    # create three new branches
                    b1 = branch(members=groups[0],id=self.id, leader=self.leader, path=[coord_a], prev=self.id)
                    time.sleep(2.0)
                    b2 = branch(members=groups[1], leader=new_leader1, path=[coord_b], prev=self.id)
                    time.sleep(2.0)
                    b3 = branch(members=groups[2], leader=new_leader2, path=[coord_c], prev=self.id)
                    time.sleep(2.0)

                    # remove the original branch from the node registry
                    #No need as one of the new branhees has the saame id
                    # self.node.remove_branch(self.id)
            else:
                self.members.remove(new_branch_leader)
                x = random.choice(frontier)
                print(x)
                new1 = branch(id=self.id, members=self.members[:n], leader=self.leader, path=[x], prev=self.id)
                branches.append(new1.id)
                frontier.remove(x)
                new2 = branch(members=self.members[n:], leader=new_branch_leader, path=[frontier[0]], prev=self.prev)
                frontier.remove(frontier[0])
                # self.node.remove_branch(self.id)
                print(f"Branch with id {self.id} split into branches {new1.id} and {new2.id}")

    def update_path(self, path):
        self.path.append(path)

    def update_branch(self, members_added):
        self.members.extend(members_added)
        for i in members_added:
            i.set_follower()
            i.follow_leader = self.leader
        print(f"Branch {self.id} updated with new members {[i.id for i in members_added]}")
        self.node.publish_branch_info(self)

    def check_leader(self, members):
        x = members[0].priority
        new_leader = members[0]
        for i in members:
            if i.priority < x:
                x = i.priority
                new_leader = i
        return new_leader

    def returning(self):
        self.retracing = True
        target_coord = self.path[0]
        self.leader.set_returning()
        for i in self.members:
            i.set_returning()
        new_path = self.path[::-1]
        
        idx = branches.index(self.prev)
        while idx >= 0:
            if branches[idx] in self.node.dict_branches:
                prev = self.node.dict_branches[branches[idx]]
                break
            idx -= 1
        
        b2 = self.node.dict_branches.get(self.prev)
        if b2:
            path_follow = b2.path
            b2.update_branch(self.members)
            self.node.remove_branch(self.id)

    def merge(self, branch_to_merge_into):
        self.members.append(self.leader)
        branch_to_merge_into.update_branch(self.members)
        self.current = branch_to_merge_into.current
        self.node.remove_branch(self.id)


def main(args=None):
    rclpy.init(args=args)
    
    # Get the universal branch_node instance
    branch_node_instance = get_branch_node()
    
    rclpy.spin(branch_node_instance)
    rclpy.shutdown()


if __name__ == '__main__':
    main()