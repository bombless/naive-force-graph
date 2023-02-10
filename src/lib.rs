use naive_graph::{Graph, NodeId, EdgeId};
use std::collections::BTreeSet;

/// Parameters to control the simulation of the force graph.
#[derive(Clone, Debug)]
pub struct SimulationParameters {
    pub force_charge: f32,
    pub force_spring: f32,
    pub force_max: f32,
    pub node_speed: f32,
    pub damping_factor: f32,
}

impl Default for SimulationParameters {
    fn default() -> Self {
        SimulationParameters {
            force_charge: 12000.0,
            force_spring: 0.3,
            force_max: 280.0,
            node_speed: 7000.0,
            damping_factor: 0.95,
        }
    }
}

/// Stores data associated with a node that can be modified by the user.
pub struct NodeData<UserNodeData = ()> {
    /// The horizontal position of the node.
    pub x: f32,
    /// The vertical position of the node.
    pub y: f32,
    /// The mass of the node.
    ///
    /// Increasing the mass of a node increases the force with which it repels other nearby nodes.
    pub mass: f32,
    /// Whether the node is fixed to its current position.
    pub is_anchor: bool,
    /// Arbitrary user data.
    ///
    /// Defaults to `()` if not specified.
    pub user_data: UserNodeData,
}

impl<UserNodeData> Default for NodeData<UserNodeData>
where
    UserNodeData: Default,
{
    fn default() -> Self {
        NodeData {
            x: 0.0,
            y: 0.0,
            mass: 10.0,
            is_anchor: false,
            user_data: Default::default(),
        }
    }
}

/// Stores data associated with an edge that can be modified by the user.
pub struct EdgeData<UserEdgeData = ()> {
    /// Arbitrary user data.
    ///
    /// Defaults to `()` if not specified.
    pub user_data: UserEdgeData,
}

impl<UserEdgeData> Default for EdgeData<UserEdgeData>
where
    UserEdgeData: Default,
{
    fn default() -> Self {
        EdgeData {
            user_data: Default::default(),
        }
    }
}

/// The main force graph structure.
pub struct ForceGraph<UserNodeData = (), UserEdgeData = ()> {
    pub parameters: SimulationParameters,
    graph: Graph<Node<UserNodeData>, EdgeData<UserEdgeData>>,
    node_indices: BTreeSet<NodeId>,
}

impl<UserNodeData, UserEdgeData> ForceGraph<UserNodeData, UserEdgeData> {
    /// Constructs a new force graph.
    ///
    /// Use the following syntax to create a graph with default parameters:
    /// ```
    /// use force_graph::ForceGraph;
    /// let graph = <ForceGraph>::new(Default::default());
    /// ```
    pub fn new(parameters: SimulationParameters) -> Self {
        ForceGraph {
            parameters,
            graph: Graph::default(),
            node_indices: Default::default(),
        }
    }

    /// Provides access to the raw graph structure if required.
    pub fn get_graph(&self) -> &Graph<Node<UserNodeData>, EdgeData<UserEdgeData>> {
        &self.graph
    }

    /// Adds a new node and returns an index that can be used to reference the node.
    pub fn add_node(&mut self, node_data: NodeData<UserNodeData>) -> NodeId {
        let idx = self.graph.add_node(Node {
            data: node_data,
            index: Default::default(),
            vx: 0.0,
            vy: 0.0,
            ax: 0.0,
            ay: 0.0,
        });
        self.graph[idx].index = Some(idx);
        self.node_indices.insert(idx);
        idx
    }

    /// Removes a node by index.
    pub fn remove_node(&mut self, idx: NodeId) {
        self.graph.remove_node(idx);
        self.node_indices.remove(&idx);
    }

    /// Removes an edge by index.
    pub fn remove_edge(&mut self, id: EdgeId) {
        self.graph.remove_edge(id);
    }

    /// Adds or updates an edge connecting two nodes by index.
    pub fn add_edge(
        &mut self,
        n1_idx: NodeId,
        n2_idx: NodeId,
        edge: EdgeData<UserEdgeData>,
    ) {
        self.graph.add_edge(n1_idx, n2_idx, edge);
    }

    /// Applies the next step of the force graph simulation.
    ///
    /// The number of seconds that have elapsed since the previous update must be calculated and
    /// provided by the user as `dt`.
    pub fn update(&mut self, dt: f32) {
        if self.graph.node_count() == 0 {
            return;
        }

        for (n1_idx_i, n1_idx) in self.node_indices.iter().enumerate() {
            let mut edges = self.graph.neighbors(*n1_idx).detach();
            while let Some(n2_idx) = edges.next_node(&self.graph) {
                let (n1, n2) = self.graph.index_twice_mut(*n1_idx, n2_idx);
                let f = attract_nodes(n1, n2, &self.parameters);
                n1.apply_force(f.0, f.1, dt, &self.parameters);
            }

            for n2_idx in self.node_indices.iter().skip(n1_idx_i + 1) {
                let (n1, n2) = self.graph.index_twice_mut(*n1_idx, *n2_idx);
                let f = repel_nodes(n1, n2, &self.parameters);
                if !n1.data.is_anchor {
                    n1.apply_force(f.0, f.1, dt, &self.parameters);
                }
                if !n2.data.is_anchor {
                    n2.apply_force(-f.0, -f.1, dt, &self.parameters);
                }
            }

            let n1 = &mut self.graph[*n1_idx];
            if !n1.data.is_anchor {
                n1.update(dt, &self.parameters);
            }
        }
    }

    /// Processes each node with a user-defined callback `cb`.
    pub fn visit_nodes<F: FnMut(&Node<UserNodeData>)>(&self, mut cb: F) {
        self.graph.visit_nodes(|_, data| cb(data))
    }

    /// Mutates each node with a user-defined callback `cb`.
    pub fn visit_nodes_mut<F: FnMut(&mut Node<UserNodeData>)>(&mut self, mut cb: F) {
        self.graph.visit_nodes_mut(|_, data| cb(data))
    }

    /// Processes each edge and its associated nodes with a user-defined callback `cb`.
    pub fn visit_edges<
        F: FnMut(&Node<UserNodeData>, &Node<UserNodeData>, &EdgeData<UserEdgeData>),
    >(
        &self,
        mut cb: F,
    ) {
        self.graph.visit_edges(|_, n1, n2, e| cb(n1, n2, e))
    }
}

/// References a node in the [ForceGraph]. Can not be constructed by the user.
pub struct Node<UserNodeData = ()> {
    /// The node data provided by the user.
    pub data: NodeData<UserNodeData>,
    index: Option<NodeId>,
    vx: f32,
    vy: f32,
    ax: f32,
    ay: f32,
}

impl<UserNodeData> Node<UserNodeData> {
    /// The horizontal position of the node.
    pub fn x(&self) -> f32 {
        self.data.x
    }

    /// The vertical position of the node.
    pub fn y(&self) -> f32 {
        self.data.y
    }

    /// The index used to reference the node in the [ForceGraph].
    pub fn index(&self) -> NodeId {
        self.index.unwrap()
    }

    fn apply_force(&mut self, fx: f32, fy: f32, dt: f32, parameters: &SimulationParameters) {
        self.ax += fx.max(-parameters.force_max).min(parameters.force_max) * dt;
        self.ay += fy.max(-parameters.force_max).min(parameters.force_max) * dt;
    }

    fn update(&mut self, dt: f32, parameters: &SimulationParameters) {
        self.vx = (self.vx + self.ax * dt * parameters.node_speed) * parameters.damping_factor;
        self.vy = (self.vy + self.ay * dt * parameters.node_speed) * parameters.damping_factor;
        self.data.x += self.vx * dt;
        self.data.y += self.vy * dt;
        self.ax = 0.0;
        self.ay = 0.0;
    }
}

fn attract_nodes<D>(n1: &Node<D>, n2: &Node<D>, parameters: &SimulationParameters) -> (f32, f32) {
    let mut dx = n2.data.x - n1.data.x;
    let mut dy = n2.data.y - n1.data.y;

    let distance = if dx == 0.0 && dy == 0.0 {
        1.0
    } else {
        (dx * dx + dy * dy).sqrt()
    };

    dx /= distance;
    dy /= distance;

    let strength = 1.0 * parameters.force_spring * distance * 0.5;
    (dx * strength, dy * strength)
}

fn repel_nodes<D>(n1: &Node<D>, n2: &Node<D>, parameters: &SimulationParameters) -> (f32, f32) {
    let mut dx = n2.data.x - n1.data.x;
    let mut dy = n2.data.y - n1.data.y;

    let distance = if dx == 0.0 && dy == 0.0 {
        1.0
    } else {
        (dx * dx + dy * dy).sqrt()
    };

    dx /= distance;
    dy /= distance;

    let distance_sqrd = distance * distance;
    let strength = -parameters.force_charge * ((n1.data.mass * n2.data.mass) / distance_sqrd);
    (dx * strength, dy * strength)
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_default() {
        let mut graph = <ForceGraph>::new(Default::default());
        let n1_idx = graph.add_node(NodeData {
            x: 0.1,
            y: 0.2,
            ..Default::default()
        });
        let n2_idx = graph.add_node(NodeData {
            x: 0.3,
            y: 0.4,
            ..Default::default()
        });
        graph.add_edge(n1_idx, n2_idx, Default::default());
    }

    #[test]
    fn test_user_data() {
        let mut graph = ForceGraph::new(Default::default());

        #[derive(Default)]
        struct UserNodeData {}
        #[derive(Default)]
        struct UserEdgeData {}

        let n1_idx = graph.add_node(NodeData {
            x: 0.1,
            y: 0.2,
            user_data: UserNodeData {},
            ..Default::default()
        });
        let n2_idx = graph.add_node(NodeData {
            x: 0.3,
            y: 0.4,
            user_data: UserNodeData {},
            ..Default::default()
        });

        graph.add_edge(
            n1_idx,
            n2_idx,
            EdgeData {
                user_data: UserEdgeData {},
            },
        );
    }
}
