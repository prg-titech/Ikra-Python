from __future__ import annotations

import os, sys, time, random, pygame, math

parentdir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
sys.path.append(parentdir + "/new-src")
from sanajeh import DeviceAllocator

kMaxNodes: int = 80
kMaxDegree: int = 5
kMaxSprings: int = kMaxNodes*kMaxDegree // 2
kMaxDistance: int = 32768
kDt: float = 0.2
kVelocityDampening: float = 0.0

kWindowWidth: int = 500
kWindowHeight: int = 500

kNumIterations: int = 100
kNumComputeIterations: int = 40

border_margin: float = 0.35
max_force: float = 0.5
spring_min: float = 3.0
spring_max: float = 5.0
mass_min: float = 500.0
mass_max: float = 500.0

num_nodes: int = int(0.65*kMaxNodes)
num_pull_nodes: int = int(0.25*kMaxNodes)
num_anchor_nodes: int = int(0.1*kMaxNodes)
num_springs: int = int(0.7*kMaxSprings)

nodes: list[NodeBase] = DeviceAllocator.array(num_nodes+num_pull_nodes+num_anchor_nodes)
dev_bfs_continue: bool = False

class NodeBase:
    def __init__(self):
        self.springs_ = [None]*kMaxDegree
        self.pos_x_: float = 0.0
        self.pos_y_: float = 0.0
        self.num_springs_: int = 0
        self.distance_: int = 0
        self.node_id_: int = 0

    def NodeBase(self, node_id: int):
        self.pos_x_ = random.uniform(border_margin, 1.0 - border_margin)
        self.pos_y_ = random.uniform(border_margin, 1.0 - border_margin)
        self.num_springs_ = 0
        self.springs_ = [None]*kMaxDegree
        self.node_id_ = node_id
        nodes[node_id] = self

    def distance_to(self, other: NodeBase) -> float:
        dx: float = self.pos_x_ - other.pos_x_
        dy: float = self.pos_y_ - other.pos_y_
        dist_sq: float = dx*dx + dy*dy
        return math.sqrt(dist_sq)

    def pos_x(self) -> float:
        return self.pos_x_
    
    def pos_y(self) -> float:
        return self.pos_y_
    
    def add_spring(self, spring: Spring):
        idx: int = self.num_springs_
        self.num_springs_ += 1
        self.springs_[idx] = spring
        assert (idx + 1) <= kMaxDegree
        assert (spring.p1() == self) or (spring.p2() == self)

    def num_springs(self) -> int:
        return self.num_springs_
    
    def remove_spring(self, spring: Spring):
        for i in range(0, kMaxDegree):
            if self.springs_[i] == spring:
                self.springs_[i] = None
                idx: int = self.num_springs_
                self.num_springs_ -= 1
                if idx == 1:
                    DeviceAllocator.destroy(self)

    def initialize_bfs(self):
        if type(self) == AnchorNode:
            self.distance_ = 0
        else:
            self.distance_ = kMaxDistance

    def bfs_visit(self, distance: int):
        if distance == self.distance_:
            dev_bfs_continue = True

            for i in range(0, kMaxDegree):
                spring: Spring = self.springs_[i]
                if spring != None:
                    n: NodeBase
                    if self == spring.p1():
                        n = spring.p2()
                    else:
                        n = spring.p1()

                    if n.distance_ == kMaxDistance:
                        n.distance_ = distance + 1

    def bfs_set_delete_flags(self):
        if self.distance_ == kMaxDistance:
            for i in range(0, kMaxDegree):
                spring: Spring = self.springs_[i]
                if spring != None:
                    spring.set_delete_flag()

class AnchorNode(NodeBase):
    def __init__(self):
        pass

    def AnchorNode(self, node_id: int):
        super().NodeBase(num_nodes + num_pull_nodes + node_id)

class AnchorPullNode(NodeBase):
    def __init__(self):
        self.vel_x_: float = 0
        self.vel_y_: float = 0
    
    def AnchorPullNode(self, node_id: int):
        super().NodeBase(num_nodes + node_id)
        self.vel_x_ = random.uniform(-0.05, 0.05)
        self.vel_y_ = random.uniform(-0.05, 0.05)

    def pull(self):
        self.pos_x_ += self.vel_x_ * kDt
        self.pos_y_ += self.vel_y_ * kDt

class Node(NodeBase):
    def __init__(self):
        self.vel_x_: float = 0
        self.vel_y_: float = 0
        self.mass_: float = 0

    def Node(self, node_id: int):
        super().NodeBase(node_id)
        self.mass_ = random.uniform(mass_min, mass_max)
        self.vel_x_ = random.uniform(border_margin, 1.0 - border_margin)
        self.vel_y_ = random.uniform(border_margin, 1.0 - border_margin)

    def move(self):
        force_x: float = 0
        force_y: float = 0

        for i in range(0, kMaxDegree):
            s: Spring = self.springs_[i]
            if s != None:
                f: NodeBase
                t: NodeBase
                if s.p1() == self:
                    f = self
                    t = s.p2()
                else:
                    assert s.p2() == self
                    f = self
                    t = s.p1()
                
                dx: float = t.pos_x() - f.pos_x()
                dy: float = t.pos_y() - f.pos_y()
                dist: float = math.sqrt(dx*dx + dy*dy)
                unit_x: float = dx/dist
                unit_y: float = dy/dist
                force_x += unit_x*(s.force())
                force_y += unit_y*(s.force())
        
        self.vel_x_ += force_x*kDt / self.mass_
        self.vel_y_ += force_y*kDt / self.mass_
        self.vel_x_ *= 1.0 - kVelocityDampening
        self.vel_y_ *= 1.0 - kVelocityDampening
        self.pos_x_ += self.vel_x_*kDt
        self.pos_y_ += self.vel_y_*kDt

class Spring:
    def __init__(self):
        self.p1_ref: NodeBase = None
        self.p2_ref: NodeBase = None
        self.spring_factor_: float = 0.0
        self.initial_length_: float = 0.0
        self.force_: float = 0.0
        self.max_force_: float = 0.0
        self.delete_flag_: bool = False

    def Spring(self, spring_id: int):
        p1_id: int = -1
        p2_id: int = -1
        while p1_id == -1:
            n: int = random.randint(0, kMaxNodes-1)
            if nodes[n].num_springs_ < kMaxDegree:
                p1_id = n
        while p2_id == -1:
            n: int = random.randint(0, kMaxNodes-1)
            if nodes[n].num_springs_ < kMaxDegree and n != p1_id:
                p2_id = n
        self.p1_ref = nodes[p1_id]
        self.p2_ref = nodes[p2_id]
        self.spring_factor_ = random.uniform(spring_min, spring_max)
        self.force_ = 0.0
        self.max_force_ = max_force
        self.initial_length_ = nodes[p1_id].distance_to(nodes[p2_id])
        self.delete_flag_ = False
        assert self.initial_length_ > 0
        self.p1_ref.add_spring(self)
        self.p2_ref.add_spring(self)

    def compute_force(self):
        dist: float = self.p1_ref.distance_to(self.p2_ref)
        displacement: float = max(0.0, dist - self.initial_length_)
        self.force_ = self.spring_factor_ * displacement
        if self.force_ > self.max_force_:
            self.self_destruct()
    
    def p1(self) -> NodeBase:
        return self.p1_ref

    def p2(self) -> NodeBase:
        return self.p2_ref

    def force(self) -> float:
        return self.force_

    def max_force(self) -> float:
        return self.max_force_

    def self_destruct(self):
        self.p1_ref.remove_spring(self)
        self.p2_ref.remove_spring(self)
        DeviceAllocator.destroy(self)

    def bfs_delete(self):
        if self.delete_flag_:
            self.self_destruct()
    
    def set_delete_flag(self):
        self.delete_flag_ = True

def main(allocator, do_render):

    def initialize_render():
        pygame.init()
        window = pygame.display.set_mode((kWindowWidth, kWindowHeight))
        screen = pygame.Surface((kWindowWidth, kWindowHeight))
        screen.fill((0,0,0))
        return (window, screen)

    def render(ob):
        x1 = int(ob.p1_ref.pos_x_ * kWindowWidth)
        x2 = int(ob.p2_ref.pos_x_ * kWindowWidth)
        y1 = int(ob.p1_ref.pos_y_ * kWindowHeight)
        y2 = int(ob.p2_ref.pos_y_ * kWindowHeight)
        pygame.draw.line(screen, (255,255,255), (x1,y1), (x2,y2), 1)
            
    allocator.initialize()
    
    allocator.parallel_new(Node, num_nodes)
    allocator.parallel_new(AnchorPullNode, num_pull_nodes)
    allocator.parallel_new(AnchorNode, num_anchor_nodes)
    allocator.parallel_new(Spring, num_springs)

    if do_render:
        os.environ["SDL_VIDEODRIVER"] = "x11"
        screen = pygame.display.set_mode((kWindowWidth, kWindowHeight))
        allocator.do_all(Spring, render)
        pygame.display.update()

    total_time = time.perf_counter()

    for i in range(kNumIterations):
        time_before = time.perf_counter()

        allocator.parallel_do(AnchorPullNode, AnchorPullNode.pull)

        for j in range(kNumComputeIterations):
            allocator.parallel_do(Spring, Spring.compute_force)
            allocator.parallel_do(Node, Node.move)

        allocator.parallel_do(NodeBase, NodeBase.initialize_bfs)

        for k in range(kMaxDistance):
            dev_bfs_continue = False
            allocator.parallel_do(NodeBase, NodeBase.bfs_visit, k)
            if not dev_bfs_continue:
                break
        
        allocator.parallel_do(NodeBase, NodeBase.bfs_set_delete_flags)
        allocator.parallel_do(Spring, Spring.bfs_delete)

        if do_render:
            screen.fill((0, 0, 0))
            allocator.do_all(Spring, render)
            pygame.display.update()

        time_after = time.perf_counter()
        total_time += time_after - time_before

    allocator.uninitialize()