use std::collections::HashSet;
use std::ops::{DerefMut, Deref};
pub use naive_graph::*;
use std::fmt::{Debug, Result as FmtRs, Formatter};

#[derive(Debug)]
pub struct Node<NodeUserData> {
    data: NodeData<NodeUserData>,
    ax: f32,
    ay: f32,
    vx: f32,
    vy: f32,
    id: Option<NodeId>,
}

#[derive(Debug, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub fn new(x: f32, y: f32) -> Self {
        Vec2 { x, y }
    }
}

impl<NodeUserData> Node<NodeUserData> {
    pub fn new(data: NodeData<NodeUserData>) -> Self {
        Self {
            data,
            ax: 0.,
            ay: 0.,
            vx: 0.,
            vy: 0.,
            id: None,
        }
    }
    pub fn index(&self) -> NodeId {
        self.id.unwrap()
    }
    pub fn x(&self) -> f32 {
        self.data.x
    }
    pub fn y(&self) -> f32 {
        self.data.y
    }
    fn apply(&mut self, force: Vec2) {
        if force.x.is_nan() {
            panic!("force.x nan")
        }
        self.ax += force.x;
        self.ay += force.y;
    }
    fn update(&mut self, parameters: &Parameters, dt: f32) {
        // println!("before {:?} {:?}", self.id.unwrap(), (self.data.x, self.data.y));
        self.data.x += parameters.spring_factor * self.vx * dt;
        self.data.y += parameters.spring_factor * self.vy * dt;
        // if self.data.x.is_nan() {
        //     println!(
        //         "parameters.spring_factor * self.vx * dt = {} * {} * {}",
        //         parameters.spring_factor, self.vx, dt
        //     );
        //     println!("self.vx = {}, self.ax = {}", self.vx, self.ax);
        // }
        // println!("after {:?} {:?}", self.id.unwrap(), (self.data.x, self.data.y));
        if self.ax.is_nan() { panic!("ax nan") }
        self.vx = self.ax;
        self.vy = self.ay;
        self.ax = 0.;
        self.ay = 0.;
    }
    pub fn user_data(&self) -> &NodeUserData {
        &self.data.user_data
    }
}

impl<NodeUserData> Deref for Node<NodeUserData> {
    type Target = NodeData<NodeUserData>;
    fn deref(&self) -> &NodeData<NodeUserData> {
        &self.data
    }
}

impl<NodeUserData> DerefMut for Node<NodeUserData> {
    fn deref_mut(&mut self) -> &mut NodeData<NodeUserData> {
        &mut self.data
    }
}

#[derive(Default, Debug)]
pub struct NodeData<NodeUserData> {
    pub user_data: NodeUserData,
    pub x: f32,
    pub y: f32,
}

pub struct Parameters {
    pub ideal_distance: f32,
    pub spring_factor: f32,
    pub count: i32,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            ideal_distance: 45.,
            spring_factor: 10.,
            count: 0,
        }
    }
}

pub struct ForceGraph<NodeUserData = (), EdgeUserData = ()> {
    graph: Graph<Node<NodeUserData>, EdgeUserData>,
    parameters: Parameters,
    nodes: HashSet<NodeId>,
}

impl<NodeUserData, EdgeUserData> Debug for ForceGraph<NodeUserData, EdgeUserData> {
    fn fmt(&self, f: &mut Formatter) -> FmtRs {
        write!(f, "{:?}", self.graph)
    }
}

impl<NodeUserData, EdgeUserData> ForceGraph<NodeUserData, EdgeUserData> {
    pub fn new(parameters: Parameters) -> Self {
        Self {
            graph: Graph::default(),
            parameters,
            nodes: HashSet::new(),
        }
    }
    pub fn node_count(&self) -> usize {
        self.graph.node_count()
    }
    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }
    pub fn add_node(&mut self, data: NodeData<NodeUserData>) -> NodeId {
        let id = self.graph.add_node(Node::new(data));
        self.graph[id].id = Some(id);
        self.nodes.insert(id);
        id
    }
    pub fn remove_node(&mut self, id: NodeId) {
        self.graph.remove_node(id);
        self.nodes.remove(&id);
    }
    pub fn add_edge(&mut self, node1: NodeId, node2: NodeId, data: EdgeUserData) -> EdgeId {
        self.graph.add_edge(node1, node2, data)
    }
    pub fn visit_edges<F: FnMut(EdgeId, &Node<NodeUserData>, &Node<NodeUserData>, &EdgeUserData)>(&self, f: F) {
        self.graph.visit_edges(f)
    }
    pub fn visit_nodes<F: FnMut(NodeId, &Node<NodeUserData>)>(&self, f: F) {
        self.graph.visit_nodes(f)
    }
    pub fn visit_nodes_mut<F: FnMut(NodeId, &mut Node<NodeUserData>)>(&mut self, f: F) {
        self.graph.visit_nodes_mut(f)
    }
    fn calculate_force(&self, diff: Vec2, distance: f32, is_neighbor: bool) -> Vec2 {

        if distance <= f32::EPSILON {
            return Vec2 { x: 0., y: 0. };
        }

        let diff_x = diff.x;
        let diff_y = diff.y;

        const FACTOR: f32 = -0.1;

        if distance < self.parameters.ideal_distance * 0.9 {
            let f_x /* = diff.x * distance / distance.powi(2) */ = diff_x * distance.powf(FACTOR);
            let f_y /* = diff_y * distance / distance.powi(2) */ = diff_y * distance.powf(FACTOR);
            
            if f_x.is_nan() {
                panic!("f.x nan")
            }

            return Vec2 {
                x: f_x,
                y: f_y,
            };
        }

        if distance > self.parameters.ideal_distance * 1.5 && is_neighbor {
            let f_x = -diff_x * distance.powf(FACTOR);
            let f_y = -diff_y * distance.powf(FACTOR);
                
            return Vec2 {
                x: f_x,
                y: f_y,
            };
        }

        Vec2 { x: 0., y: 0. }        
    }
    pub fn update(&mut self, dt: f32) {
        fn bounce(really_close_distance: f32) -> Vec2 {            
            let x = rand::random::<f32>() * really_close_distance;
            let y = (1. - x.powi(2)).sqrt() * really_close_distance;
            Vec2 { x, y }
        }
        if self.parameters.count <= 100 {
            self.parameters.count += 1;
        }
        let really_close_distance = self.parameters.ideal_distance / 10000.;
        let mut bouncing = None;
        for &m in &self.nodes {
            let m_neighbors = self.graph.neighbor_id_set(m);
            if self.parameters.count < 100 {
                //println!("neighbors {:?}", m_neighbors.len())
            }
            for &n in &self.nodes {
                if m == n { continue }
                let dst = &self.graph[m].data;
                let src = &self.graph[n].data;
                let diff = Vec2 { x: dst.x - src.x, y: dst.y - src.y, };
                let distance = (diff.x.powi(2) + diff.y.powi(2)).sqrt();

                if distance < really_close_distance && bouncing.is_none() {
                    bouncing = Some((m, bounce(really_close_distance)));
                    continue;
                }

                let f = self.calculate_force(diff, distance, m_neighbors.contains(&n));
                if self.parameters.count < 100 {
                    //println!("calculate_force {:?}", f)
                }
                self.graph[m].apply(f);
            }
            self.graph[m].update(&self.parameters, dt);
        }
        if let Some((id, b)) = bouncing {
            // println!("before {:?}", (self.graph[id].x, self.graph[id].y));
            self.graph[id].x += b.x;
            self.graph[id].y += b.y;
            // println!("after {:?}", (self.graph[id].x, self.graph[id].y));
        }
    }
    pub fn visit_intersections<F: FnMut(IntersectionInfo)>(&self, mut f: F) {
        fn get_line_intersection((p0, p1): &(Vec2, Vec2), (p2, p3): &(Vec2, Vec2)) -> Option<Vec2>
        {
            let s1 = Vec2 { x: p1.x - p0.x, y: p1.y - p0.y };
            let s2 = Vec2 { x: p3.x - p2.x, y: p3.y - p2.y };
        
            let s = (-s1.y * (p0.x - p2.x) + s1.x * (p0.y - p2.y)) / (-s2.x * s1.y + s1.x * s2.y);
            let t = ( s2.x * (p0.y - p2.y) - s2.y * (p0.x - p2.x)) / (-s2.x * s1.y + s1.x * s2.y);
        
            if s >= 0. && s <= 1. && t >= 0. && t <= 1. {
                // Collision detected
                return Some(Vec2 { x: p0.x + (t * s1.x), y: p0.y + (t * s1.y) });
            }
        
            return None; // No collision
        }
        let mut lines = Vec::new();
        self.graph.visit_edges(|_, n1, n2, _| {
            let from = Vec2 { x: n1.x(), y: n1.y() };
            let to = Vec2 { x: n2.x(), y: n2.y() };

            lines.push((from, to));
        });

        for line1 in &lines {
            for line2 in &lines {
                if line1 == line2 {
                    continue;
                }
                if let Some(Vec2 { x, y }) = get_line_intersection(line1, line2) {
                    f(IntersectionInfo { x, y })
                }
            }
        }

    }
}

pub struct IntersectionInfo {
    x: f32,
    y: f32,
}

impl IntersectionInfo {
    pub fn x(&self) -> f32 {
        self.x
    }
    pub fn y(&self) -> f32 {
        self.y
    }
}
