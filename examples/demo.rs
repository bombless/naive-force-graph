use naive_force_graph::{ForceGraph, NodeData, Node};
use macroquad::prelude::*;

const NODE_RADIUS: f32 = 15.0;

#[macroquad::main("Demo")]
async fn main() {
    // create a force graph with default parameters
    let mut graph = ForceGraph::<i32>::new(Default::default());

    // let n1_idx = graph.add_node(NodeData {
    //     x: 250.0,
    //     y: 250.0,
    //     ..Default::default()
    // });
    // let n2_idx = graph.add_node(NodeData {
    //     x: 750.0,
    //     y: 250.0,
    //     ..Default::default()
    // });
    // let n3_idx = graph.add_node(NodeData {
    //     x: 250.0,
    //     y: 750.0,
    //     ..Default::default()
    // });
    // let n4_idx = graph.add_node(NodeData {
    //     x: 750.0,
    //     y: 750.0,
    //     ..Default::default()
    // });
    // let n5_idx = graph.add_node(NodeData {
    //     x: 500.0,
    //     y: 500.0,
    //     ..Default::default()
    // });

    // // set up links between nodes
    // // graph.add_edge(n1_idx, n2_idx, Default::default());
    // graph.add_edge(n1_idx, n5_idx, Default::default());
    // graph.add_edge(n2_idx, n5_idx, Default::default());
    // graph.add_edge(n3_idx, n5_idx, Default::default());
    // graph.add_edge(n4_idx, n5_idx, Default::default());

    let n1 = graph.add_node(NodeData { user_data: 1, x: 100., y: 100., ..NodeData::default() });
    let n2 = graph.add_node(NodeData { user_data: 2, x: 100., y: 100., ..NodeData::default() });
    let n3 = graph.add_node(NodeData { user_data: 3, x: 100., y: 100., ..NodeData::default() });
    let n4 = graph.add_node(NodeData { user_data: 4, x: 100., y: 100., ..NodeData::default() });

    graph.add_edge(n1, n2, Default::default());
    graph.add_edge(n1, n3, Default::default());
    graph.add_edge(n4, n2, Default::default());
    graph.add_edge(n4, n3, Default::default());

    let mut i = 0;

    let mut dragging_node_idx = None;
    loop {
        clear_background(BLACK);
        draw_text(
            "Drag nodes with the left mouse button",
            50.0,
            50.0,
            25.0,
            WHITE,
        );

        // draw edges
        graph.visit_edges(|_, node1, node2, _edge| {
            if i < 100 {
                println!("node1 {:?} [{} {}]", node1.index(), node1.x(), node1.y());
                println!("node2 {:?} [{} {}]", node2.index(), node2.x(), node2.y());
            }
            draw_line(node1.x(), node1.y(), node2.x(), node2.y(), 2.0, GRAY);
        });

        graph.visit_intersections(|info| {
            draw_circle(info.x(), info.y(), 2.0, RED);
        });

        // draw nodes
        graph.visit_nodes(|_, node| {
            if i < 100 {
                println!("{:?} [{} {}]", node.index(), node.x(), node.y());
            }
            draw_circle(node.x(), node.y(), NODE_RADIUS, WHITE);

            // highlight hovered or dragged node
            if node_overlaps_mouse_position(node)
                || dragging_node_idx
                    .filter(|idx| *idx == node.index())
                    .is_some()
            {
                draw_circle_lines(node.x(), node.y(), NODE_RADIUS, 2.0, RED);
            }

            draw_text(
                &node.user_data().to_string(),
                node.x(),
                node.y(),
                25.0,
                BLUE,
            );
        });

        // drag nodes with the mouse
        if is_mouse_button_down(MouseButton::Left) {
            graph.visit_nodes_mut(|_, node| {
                if let Some(idx) = dragging_node_idx {
                    if idx == node.index() {
                        let (mouse_x, mouse_y) = mouse_position();
                        node.x = mouse_x;
                        node.y = mouse_y;
                    }
                } else if node_overlaps_mouse_position(node) {
                    dragging_node_idx = Some(node.index());
                }
            });
        } else {
            dragging_node_idx = None;
        }

        if i <= 100 {
            i += 1;
        }

        graph.update(get_frame_time());

        next_frame().await
    }
}

fn node_overlaps_mouse_position<NodeUserData>(node: &Node<NodeUserData>) -> bool {
    let (mouse_x, mouse_y) = mouse_position();
    ((node.x() - mouse_x) * (node.x() - mouse_x) + (node.y() - mouse_y) * (node.y() - mouse_y))
        < NODE_RADIUS * NODE_RADIUS
}
