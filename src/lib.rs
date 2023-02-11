use naive_graph::{NodeId, EdgeId, Graph};
use std::collections::HashSet;
use std::ops::{DerefMut, Deref};

#[derive(Debug)]
pub struct Node<NodeUserData> {
    data: NodeData<NodeUserData>,
    ax: f32,
    ay: f32,
    vx: f32,
    vy: f32,
    id: Option<NodeId>,
}

#[derive(Debug)]
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
        self.ax += force.x;
        self.ay += force.y;
    }
    fn update(&mut self, parameters: &Parameters, dt: f32) {
        self.data.x += parameters.spring_factor * self.vx * dt;
        self.data.y += parameters.spring_factor * self.vy * dt;
        self.vx = self.ax * dt;
        self.vy = self.ay * dt;
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
            spring_factor: 1000.,
            count: 0,
        }
    }
}

pub struct ForceGraph<NodeUserData = (), EdgeUserData = ()> {
    graph: Graph<Node<NodeUserData>, EdgeUserData>,
    parameters: Parameters,
    nodes: HashSet<NodeId>,
}

impl<NodeUserData, EdgeUserData> ForceGraph<NodeUserData, EdgeUserData> {
    pub fn new(parameters: Parameters) -> Self {
        Self {
            graph: Graph::default(),
            parameters,
            nodes: HashSet::new(),
        }
    }
    pub fn add_node(&mut self, data: NodeData<NodeUserData>) -> NodeId {
        let id = self.graph.add_node(Node::new(data));
        self.graph[id].id = Some(id);
        self.nodes.insert(id);
        id
    }
    pub fn add_edge(&mut self, node1: NodeId, node2: NodeId, data: EdgeUserData) -> EdgeId {
        self.graph.add_edge(node1, node2, data)
    }
    pub fn visit_edges<F: Fn(EdgeId, &Node<NodeUserData>, &Node<NodeUserData>, &EdgeUserData)>(&self, f: F) {
        self.graph.visit_edges(f)
    }
    pub fn visit_nodes<F: Fn(NodeId, &Node<NodeUserData>)>(&self, f: F) {
        self.graph.visit_nodes(f)
    }
    pub fn visit_nodes_mut<F: FnMut(NodeId, &mut Node<NodeUserData>)>(&mut self, f: F) {
        self.graph.visit_nodes_mut(f)
    }
    fn calculate_force(&self, dst: Vec2, src: Vec2, is_neighbor: bool) -> Vec2 {
        let diff_x = dst.x - src.x;
        let diff_y = dst.y - src.y;
        let distance = (diff_x.powi(2) + diff_y.powi(2)).sqrt();

        const FACTOR: f32 = -0.1;

        if distance < self.parameters.ideal_distance * 0.9 {
            let f_x /* = diff_x * distance / distance.powi(2) */ = diff_x * distance.powf(FACTOR);
            let f_y /* = diff_y * distance / distance.powi(2) */ = diff_y * distance.powf(FACTOR);
            
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
        if self.parameters.count <= 100 {
            self.parameters.count += 1;
        }
        for &m in &self.nodes {
            let m_neighbors = self.graph.neighbor_id_set(m);
            if self.parameters.count < 100 {
                println!("neighbors {:?}", m_neighbors.len())
            }
            for &n in &self.nodes {
                if m == n { continue }
                let m_data = &self.graph[m].data;
                let m_pos = Vec2 { x: m_data.x, y: m_data.y };
                let n_data = &self.graph[n].data;
                let n_pos = Vec2 { x: n_data.x, y: n_data.y };
                let f = self.calculate_force(m_pos, n_pos, m_neighbors.contains(&n));
                if self.parameters.count < 100 {
                    //println!("calculate_force {:?}", f)
                }
                self.graph[m].apply(f);
            }
            self.graph[m].update(&self.parameters, dt);
        }
    }
}
